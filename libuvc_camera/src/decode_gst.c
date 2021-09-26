/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (C) 2021 Koji Takeo
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include <assert.h>
#include <stdlib.h>
#define __USE_GNU
#include <pthread.h>

#include <libuvc/libuvc.h>

#include <gst/gst.h>
#include <gst/gstbuffer.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>

#include "libuvc_camera/decode_gst.h"

#if defined(DECODER_JETSON)
#if defined(USE_OPENGL)
#define DECODE_PIPE "nvv4l2decoder ! nvvidconv ! queue ! glupload ! glcolorconvert ! video/x-raw(memory:GLMemory),format=RGB ! gldownload"
#else
#define DECODE_PIPE "nvv4l2decoder ! nvvidconv ! video/x-raw,format=RGBA ! queue ! videoconvert n-threads=2 !  video/x-raw,format=RGB ! queue "
#endif
#else
#define DECODE_PIPE "decodebin ! glupload ! glcolorconvert ! video/x-raw(memory:GLMemory),format=RGB ! gldownload"
#endif
#define PIPE_FORMAT "appsrc name=src ! queue ! h264parse ! %s ! queue ! appsink drop=true name=sink"

enum msg_code {
    MSG_READY = 0,
    MSG_NEW_DATA,
    MSG_TERMINATE
};

struct publish_thr_ctl {
    pthread_t thr;
    pthread_mutex_t lock;
    pthread_cond_t cond;
    enum msg_code msg;
    void *ptr;
};


struct decode_gst {
    GstElement *pipeline;
    GstElement *src;
    GstElement *sink;
    GMainLoop *loop;

    struct publish_thr_ctl ptc;
    pthread_t decode_thr;
    unsigned int fcount;

    char *pipeline_desc;

    void (*driver_cb)(uvc_frame_t *, void *);
    void *cb_arg;

    uvc_frame_t out_frame;
};


void
decoder_cb(uvc_frame_t *frame, void *arg)
{
    decode_gst_t *p;
    GstBuffer *buffer;
    GstFlowReturn ret;
    GstMapInfo map;

    p = (decode_gst_t *)arg;
    buffer = gst_buffer_new_allocate(NULL, frame->data_bytes, NULL);
    GST_BUFFER_PTS(buffer) = GST_CLOCK_TIME_NONE;
    GST_BUFFER_DTS(buffer) = GST_CLOCK_TIME_NONE;
    GST_BUFFER_DURATION(buffer) = GST_CLOCK_TIME_NONE;

    gst_buffer_map(buffer, &map, GST_MAP_WRITE);
    memcpy(map.data, frame->data, frame->data_bytes);
    gst_buffer_unmap(buffer, &map);

    g_signal_emit_by_name(p->src, "push-buffer", buffer, &ret);
    gst_buffer_unref(buffer);

    return;
}

static GstFlowReturn
newsample_cb(GstElement *e, decode_gst_t *p)
{
    GstBuffer *buffer, *oldbuffer;
    GstMapInfo map;
    GstSample *sample;

    sample = gst_app_sink_pull_sample(GST_APP_SINK(e));
    buffer = gst_sample_get_buffer(sample);

    oldbuffer = NULL;
    p->fcount++;

    pthread_mutex_lock(&(p->ptc.lock));
    if (p->ptc.msg == MSG_NEW_DATA) {
	fprintf(stderr, "Frame skipped %8d\n", p->fcount);
	oldbuffer = p->ptc.ptr;
    }
    p->ptc.msg = MSG_NEW_DATA;
    p->ptc.ptr = buffer;
    gst_buffer_ref(buffer);
    pthread_cond_signal(&(p->ptc.cond));

    pthread_mutex_unlock(&(p->ptc.lock));
    gst_sample_unref(sample);
    if (oldbuffer)
	gst_buffer_unref(oldbuffer);

    return GST_FLOW_OK;
}

void *
publish_loop(void *arg)
{
    decode_gst_t *p;
    GstBuffer *buffer;
    GstMapInfo map;
    enum msg_code msg;
    int cont;

    p = (decode_gst_t *)arg;

    p->ptc.msg = MSG_READY;

    cont = 1;
    while (cont) {
	pthread_mutex_lock(&(p->ptc.lock));
	while (p->ptc.msg == MSG_READY)
	    pthread_cond_wait(&(p->ptc.cond), &(p->ptc.lock));
	msg = p->ptc.msg;

	buffer = p->ptc.ptr;
	p->ptc.msg = MSG_READY;
	pthread_mutex_unlock(&(p->ptc.lock));

	switch (msg) {
	case MSG_NEW_DATA:
	    gst_buffer_map(buffer, &map, GST_MAP_READ);
	    p->out_frame.data = map.data;
	    p->driver_cb(&(p->out_frame), p->cb_arg);
	    gst_buffer_unmap(buffer, &map);
	    gst_buffer_unref(buffer);
	    break;
	case MSG_TERMINATE:
	    cont = 0;
	    break;
	default:
	    break;
	}
    }

}

void *
decoder_loop(void *arg)
{
    GstCaps *caps;
    decode_gst_t *p;

    p = (decode_gst_t *)arg;

    gst_init(NULL, NULL);

    p->loop = g_main_loop_new(NULL, TRUE);
    p->pipeline = gst_parse_launch(p->pipeline_desc, NULL);

    if (p->pipeline == NULL)
	return NULL;

    gst_pipeline_set_clock(GST_PIPELINE(p->pipeline), gst_system_clock_obtain());

    p->src = gst_bin_get_by_name(GST_BIN(p->pipeline), "src");
    caps = gst_caps_new_simple("video/x-h264",
	"framerate", GST_TYPE_FRACTION, 30000, 1001,
	"stream-format", G_TYPE_STRING, "byte-stream", NULL);
    gst_app_src_set_caps(GST_APP_SRC(p->src), caps);

    p->sink = gst_bin_get_by_name(GST_BIN(p->pipeline), "sink");
    g_object_set(G_OBJECT(p->sink), "emit-signals", TRUE, "sync", TRUE, NULL);
    g_signal_connect(GST_APP_SINK(p->sink), "new-sample", G_CALLBACK(newsample_cb), p);

    
    gst_element_set_state(p->pipeline, GST_STATE_PLAYING);
    g_main_loop_run(p->loop);

    gst_element_set_state(p->pipeline, GST_STATE_NULL);
    g_main_loop_unref(p->loop);
}

decode_gst_t *
decode_gst_init(void (*cb)(uvc_frame_t *, void *), void *arg, const char *decoder,
		size_t width, size_t height)
{
    char *pipeline_desc;
    decode_gst_t *p;


    p = (decode_gst_t *)malloc(sizeof(decode_gst_t));
    if (p == NULL)
	return NULL;

    p->driver_cb = cb;
    p->cb_arg = arg;

    memset(&(p->out_frame), sizeof(uvc_frame_t), 0);
    p->out_frame.width = width;
    p->out_frame.height = height;
    p->out_frame.data_bytes = width * height * 3;
    p->out_frame.frame_format = UVC_FRAME_FORMAT_RGB;
    p->fcount = 0;

    if (decoder == NULL)
	decoder = DECODE_PIPE;

    pipeline_desc = (char *)malloc(strlen(decoder) + strlen(PIPE_FORMAT));
    sprintf(pipeline_desc, PIPE_FORMAT, decoder);
    p->pipeline_desc = pipeline_desc;
    pthread_create(&(p->decode_thr), NULL, decoder_loop, p);

    pthread_cond_init(&(p->ptc.cond), NULL);
    pthread_mutex_init(&(p->ptc.lock), NULL);
    pthread_create(&(p->ptc.thr), NULL, publish_loop, p);

    pthread_setname_np(p->ptc.thr, "publish");

    return p;
}

void
decode_gst_terminate(decode_gst_t *p)
{
    pthread_mutex_lock(&(p->ptc.lock));
    p->ptc.msg = MSG_TERMINATE;
    pthread_cond_signal(&(p->ptc.cond));
    pthread_mutex_unlock(&(p->ptc.lock));

    g_main_loop_quit(p->loop);

    pthread_join(p->decode_thr, NULL);
    pthread_join(p->ptc.thr, NULL);

    free(p->pipeline_desc);
    free(p);
}
