#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "MvCameraControl.h"
#include "MvErrorDefine.h"
#include "CameraParams.h"

#define MAX_IMAGE_DATA_SIZE (3 * 2160 * 2600)

void *handle = NULL;
uint8_t *m_pBufForSaveImage;                    // 转换后图像数据缓存
MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0}; // 像素格式转换参数
ros::Publisher image_pub;

// 图像数据回调函数
void __stdcall ImageCallback(unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser) 
{
    if (pFrameInfo == nullptr) return;

    stConvertParam.nWidth = pFrameInfo->nWidth;               // ch:图像宽 | en:image width
    stConvertParam.nHeight = pFrameInfo->nHeight;             // ch:图像高 | en:image height
    stConvertParam.enSrcPixelType = pFrameInfo->enPixelType;  // ch:输入像素格式 | en:input pixel format
    stConvertParam.pSrcData = pData;                          // ch:输入数据缓存 | en:input data buffer
    MV_CC_ConvertPixelType(handle, &stConvertParam);          // 转换图像格式为BGR8

    // ROS_INFO("Receive image at %ld\n", pFrameInfo->nHostTimeStamp);

    // 将OpenCV图像转换为ROS图像消息
    cv::Mat frame(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC3, m_pBufForSaveImage);
    sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

    uint64_t sec = pFrameInfo->nHostTimeStamp / 1000;
    uint64_t nsec = (pFrameInfo->nHostTimeStamp % 1000) * 1000000;
    image_msg->header.stamp.sec = sec;
    image_msg->header.stamp.nsec = nsec;
    image_msg->header.frame_id = "camera_frame";

    // 发布图像和相机信息
    image_pub.publish(image_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hikrobot_camera");
    ros::NodeHandle nh;

    int width;                // 图像宽度
    int height;               // 图像高度
    int offset_x;             // 图像偏移量（X 轴）
    int offset_y;             // 图像偏移量（Y 轴）
    bool frameRateEnable;     // 帧率启用开关
    int frameRate;            // 配置的帧率值
    int exposureTime;         // 曝光时间设置（单位：微秒）
    bool gammaEnable;         // 伽马校正启用开关
    float gamma;              // 伽马值
    int triggerMode;          // 触发模式
    int triggerSource;        // 触发源

    // 创建图像发布器
    image_pub = nh.advertise<sensor_msgs::Image>("/hikrobot_camera/rgb", 10);
    
    // 转换后图像数据缓存
    m_pBufForSaveImage = (unsigned char *)malloc(MAX_IMAGE_DATA_SIZE); 

    stConvertParam.nSrcDataLen = MAX_IMAGE_DATA_SIZE;           // ch:输入数据大小 | en:input data size
    stConvertParam.pDstBuffer = m_pBufForSaveImage;             // ch:输出数据缓存 | en:output data buffer
    stConvertParam.nDstBufferSize = MAX_IMAGE_DATA_SIZE;        // ch:输出缓存大小 | en:output buffer size
    stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed; // ch:输出像素格式 | en:output pixel format

    // 初始化 SDK
    int nRet = MV_CC_Initialize();
    if (nRet != MV_OK) {
        ROS_ERROR("Failed to initialize SDK! Error code: %d", nRet);
        return -1;
    }

    // 枚举设备
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (nRet != MV_OK || stDeviceList.nDeviceNum == 0) {
        ROS_ERROR("No device found or enumeration failed! Error code: %d", nRet);
        MV_CC_Finalize();
        return -1;
    }

    // 创建句柄
    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[0]);
    if (nRet != MV_OK) {
        ROS_ERROR("Failed to create device handle! Error code: %d", nRet);
        MV_CC_Finalize();
        return -1;
    }

    // 打开设备
    nRet = MV_CC_OpenDevice(handle);
    if (nRet != MV_OK) {
        ROS_ERROR("Failed to open device! Error code: %d", nRet);
        MV_CC_DestroyHandle(handle);
        MV_CC_Finalize();
        return -1;
    }
    
    // 读取摄像头参数配置，如果未提供，则使用默认值
    nh.param("Width", width, 2600);                      // 图像宽度
    nh.param("Height", height, 2160);                    // 图像高度
    nh.param("Offset_x", offset_x, 0);                   // 图像偏移量（X 轴）
    nh.param("Offset_y", offset_y, 0);                   // 图像偏移量（Y 轴）
    nh.param("FrameRateEnable", frameRateEnable, false); // 帧率启用开关
    nh.param("FrameRate", frameRate, 20);                // 帧率值
    nh.param("ExposureTime", exposureTime, 40000);       // 曝光时间（单位：微秒）
    nh.param("GammaEnable", gammaEnable, false);         // 伽马校正启用开关
    nh.param("Gamma", gamma, (float)0.7);                // 伽马值
    nh.param("TriggerMode", triggerMode, 1);             // 触发模式
    nh.param("TriggerSource", triggerSource, 0);         // 触发源

    // 设置图像宽度和高度
    nRet = MV_CC_SetIntValueEx(handle, "Width", width);
    if (nRet != MV_OK) {
        ROS_ERROR("Failed to set image width! Error code: %d", nRet);
    }

    nRet = MV_CC_SetIntValueEx(handle, "Height", height);
    if (nRet != MV_OK) {
        ROS_ERROR("Failed to set image height! Error code: %d", nRet);
    }

    // 设置图像偏移量
    nRet = MV_CC_SetIntValueEx(handle, "OffsetX", offset_x);
    if (nRet != MV_OK) {
        ROS_ERROR("Failed to set offset X! Error code: %d", nRet);
    }

    nRet = MV_CC_SetIntValueEx(handle, "OffsetY", offset_y);
    if (nRet != MV_OK) {
        ROS_ERROR("Failed to set offset Y! Error code: %d", nRet);
    }

    // 设置帧率
    nRet = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", frameRateEnable);
    if (nRet != MV_OK) {
        ROS_ERROR("Failed to set frame rate enable! Error code: %d", nRet);
    }
    if (frameRateEnable) {
        nRet = MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", frameRate);
        if (nRet != MV_OK) {
            ROS_ERROR("Failed to set frame rate! Error code: %d", nRet);
        }
    }

    // 设置曝光时间
    nRet = MV_CC_SetFloatValue(handle, "ExposureTime", exposureTime);
    if (nRet != MV_OK) {
        ROS_ERROR("Failed to set exposure time! Error code: %d", nRet);
    }
    
    // 设置伽马
    nRet = MV_CC_SetBoolValue(handle, "GammaEnable", gammaEnable);
    if (nRet != MV_OK) {
        ROS_ERROR("Failed to set gamma enable! Error code: %d", nRet);
    }
    if (gammaEnable) {
        nRet = MV_CC_SetFloatValue(handle, "Gamma", gamma);
        if (nRet != MV_OK) {
            ROS_ERROR("Failed to set gamma! Error code: %d", nRet);
        }
    }

    // 设置触发模式
    nRet = MV_CC_SetEnumValue(handle, "TriggerMode", triggerMode);
    if (nRet != MV_OK) {
        ROS_ERROR("Failed to set trigger mode! Error code: %d", nRet);
    }

    // 设置触发源
    nRet = MV_CC_SetEnumValue(handle, "TriggerSource", triggerSource);
    if (nRet != MV_OK) {
        ROS_ERROR("Failed to set trigger source! Error code: %d", nRet);
    }

    // 注册图像回调
    nRet = MV_CC_RegisterImageCallBackEx(handle, ImageCallback, NULL);
    if (nRet != MV_OK) {
        ROS_ERROR("Failed to register callback! Error code: %d", nRet);
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        MV_CC_Finalize();
        return -1;
    }

    // 开始取流
    nRet = MV_CC_StartGrabbing(handle);
    if (nRet != MV_OK) {
        ROS_ERROR("Failed to start grabbing! Error code: %d", nRet);
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        MV_CC_Finalize();
        return -1;
    }

    while(ros::ok());

    // 停止取流并释放资源
    MV_CC_StopGrabbing(handle);
    MV_CC_CloseDevice(handle);
    MV_CC_DestroyHandle(handle);
    MV_CC_Finalize();

    free(m_pBufForSaveImage);

    ROS_INFO("Camera node finished.");
    return 0;
}
