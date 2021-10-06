ROS Driver for USB Video Class Cameras
======================================

`libuvc_camera` is a ROS driver that supports webcams and other UVC-standards-compliant video devices.
It's a cross-platform replacement for `uvc_camera`, a Linux-only webcam driver.

Documentation is available on the ROS wiki: [libuvc_camera](http://wiki.ros.org/libuvc_camera).


---
## H.264 support
H.264 support(as video_mode:=h264) via gstreamer.<br>
Need gstreamer pulgin/development pacakges to build and run.

* If you installed libuvc from source, check [here](https://github.com/nickel110/libuvc/commit/8b58a694e4cdedd6dc09031398e927c3092f1b70) for fix cmake module file.
* Tested on NVIDIA Jetson and Intel with iGPU platforms.

### Example
Capture 2K image from THETA V
```
$ rosrun  libuvc_camera camera_node  _vendor:=0x05ca _product:=0x2712 _width:=1920 _height:=960 _frame_rate:=29.97 _video_mode:=h264
```
To reduce the system workload, you can reduce frame rate to 1/n of the original (where n is a natural number).
```
$ rosrun  libuvc_camera camera_node  _vendor:=0x05ca _product:=0x2712 _width:=1920 _height:=960 _frame_rate:=29.97 _video_mode:=h264 _frame_rate_reduction_ratio:=2
```
or, you can scale down output image size.
```
$ rosrun  libuvc_camera camera_node  _vendor:=0x05ca _product:=0x2712 _width:=1920 _height:=960 _frame_rate:=29.97 _video_mode:=h264 _out_width:=1280 _out_height:=640
```
