# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(libuvc_camera)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp camera_info_manager dynamic_reconfigure image_transport nodelet sensor_msgs)

generate_dynamic_reconfigure_options(cfg/UVCCamera.cfg)

find_package(libuvc REQUIRED)
message(STATUS "libuvc ${libuvc_VERSION_MAJOR}.${libuvc_VERSION_MINOR}.${libuvc_VERSION_PATCH}")

find_package(PkgConfig)
pkg_search_module(GLIB REQUIRED glib-2.0)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    camera_info_manager
    dynamic_reconfigure
    image_transport
    nodelet
    sensor_msgs
  LIBRARIES libuvc_camera_nodelet
  )

add_definitions(-Dlibuvc_VERSION_MAJOR=${libuvc_VERSION_MAJOR})
add_definitions(-Dlibuvc_VERSION_MINOR=${libuvc_VERSION_MINOR})
add_definitions(-Dlibuvc_VERSION_PATCH=${libuvc_VERSION_PATCH})
include_directories(include ${libuvc_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GST_APP_INCLUDE_DIRS})
link_directories(${catkin_LINK_DIRS})

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

add_executable(camera_node src/main.cpp src/camera_driver.cpp src/decode_gst.c)
target_link_libraries(camera_node ${libuvc_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${GST_APP_LIBRARIES})
add_dependencies(camera_node ${PROJECT_NAME}_gencfg)

add_library(libuvc_camera_nodelet src/nodelet.cpp src/camera_driver.cpp src/decode_gst.c)
add_dependencies(libuvc_camera_nodelet ${libuvc_camera_EXPORTED_TARGETS})
target_link_libraries(libuvc_camera_nodelet ${libuvc_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES}
  ${GST_APP_LIBRARIES})
add_dependencies(libuvc_camera_nodelet ${PROJECT_NAME}_gencfg)

install(TARGETS camera_node libuvc_camera_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES libuvc_camera_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
