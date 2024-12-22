cmake_minimum_required(VERSION 2.8.3)

project(hikrobot_camera)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  camera_info_manager
)

catkin_package(
 
)

find_package(OpenCV 3.2.0 REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
  /opt/MVS/include/
)

LINK_DIRECTORIES(
  /opt/MVS/lib/aarch64/
)

add_executable(${PROJECT_NAME}
src/hikrobot_camera.cpp)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  # GCBase_gcc421_v3_0
  # GenApi_gcc421_v3_0
  # log4cpp_gcc421_v3_0
  # Log_gcc421_v3_0
  # MathParser_gcc421_v3_0
  # NodeMapData_gcc421_v3_0
  # XmlParser_gcc421_v3_0
  MvCameraControl
  MVGigEVisionSDK
  MVRender
  MvUsb3vTL
  MediaProcess
  X11
)