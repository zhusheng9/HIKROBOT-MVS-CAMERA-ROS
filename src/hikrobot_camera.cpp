#include <iostream>
#include "opencv2/opencv.hpp"
#include <vector>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include "hikrobot_camera.hpp"

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    cv::Mat src;
    ros::init(argc, argv, "hikrobot_camera");
    ros::NodeHandle hikrobot_camera;
    camera::Camera MVS_cap(hikrobot_camera, "/hikrobot_camera/rgb");
    
    MVS_cap.HKRun();

    return 0;
}
