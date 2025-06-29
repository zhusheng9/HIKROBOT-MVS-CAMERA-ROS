#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "MvCameraControl.h"
#include "MvErrorDefine.h"
#include "CameraParams.h"

#include <fcntl.h>        // open, O_RDWR
#include <unistd.h>       // close, read, write
#include <sys/mman.h>     // mmap, PROT_READ, PROT_WRITE, MAP_SHARED, MAP_FAILED
#include <sys/stat.h>     // optional: fstat
#include <sys/types.h>    // optional: off_t, size_t

#define MAX_IMAGE_DATA_SIZE (3 * 2160 * 2600)

struct time_stamp {
  int64_t high;
  int64_t low;
};
time_stamp *pointt;
void *handle = NULL;
uint8_t *m_pBufForSaveImage;                    // 转换后图像数据缓存
MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0}; // 像素格式转换参数
image_transport::Publisher image_pub;

// 图像数据回调函数
void __stdcall ImageCallback(unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser) 
{
    if (pFrameInfo == nullptr) return;

    ros::Time rcv_time;
    if (pointt != MAP_FAILED && pointt->low != 0) {
        int64_t b = pointt->low;
        double time_pc = b / 1000000000.0;
        rcv_time = ros::Time(time_pc);
    } else {
        rcv_time = ros::Time::now();
    }

    stConvertParam.nWidth = pFrameInfo->nWidth;
    stConvertParam.nHeight = pFrameInfo->nHeight;
    stConvertParam.enSrcPixelType = pFrameInfo->enPixelType;
    stConvertParam.pSrcData = pData;
    MV_CC_ConvertPixelType(handle, &stConvertParam);

    // ROS_INFO("Receive image at %ld\n", pFrameInfo->nHostTimeStamp);

    // 将OpenCV图像转换为ROS图像消息
    cv::Mat frame(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC3, m_pBufForSaveImage);
    sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

    // image_msg->header.stamp.sec = pFrameInfo->nHostTimeStamp / 1000;
    // image_msg->header.stamp.nsec = (pFrameInfo->nHostTimeStamp % 1000) * 1000000;
    image_msg->header.stamp = rcv_time;
    image_msg->header.frame_id = "hikrobot_camera";

    // 发布图像和相机信息
    image_pub.publish(image_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hikrobot_camera");
    ros::NodeHandle nh;

    int Width;
    int Height;
    int BinningHorizontal;
    int BinningVertical;
    float AcquisitionFrameRate;
    bool AcquisitionFrameRateEnable;
    int TriggerMode;
    int TriggerSource;
    int TriggerActivation;
    int ExposureTime;
    int ExposureAuto;
    int AutoExposureTimeLowerLimit;
    int AutoExposureTimeUpperLimit;
    int BlackLevel;
    bool BlackLevelEnable;
    int BalanceWhiteAuto;
    float Gamma;
    bool GammaEnable;

    // 创建图像发布器
    image_transport::ImageTransport it(nh);
    image_pub = it.advertise("/hikrobot_camera/rgb", 1);
    
    // 转换后图像数据缓存
    m_pBufForSaveImage = (unsigned char *)malloc(MAX_IMAGE_DATA_SIZE); 

    stConvertParam.nSrcDataLen = MAX_IMAGE_DATA_SIZE;
    stConvertParam.pDstBuffer = m_pBufForSaveImage;
    stConvertParam.nDstBufferSize = MAX_IMAGE_DATA_SIZE;
    stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

    const char *user_name = getlogin();
    std::string path_for_time_stamp = "/home/" + std::string(user_name) + "/timeshare";
    const char *shared_file_name = path_for_time_stamp.c_str();
    int fd = open(shared_file_name, O_RDWR);
    pointt = (time_stamp *)mmap(NULL, sizeof(time_stamp), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

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
    
    // 读取参数
    nh.param("Width", Width, 2600);
    nh.param("Height", Height, 2160);
    nh.param("BinningHorizontal", BinningHorizontal, 1);
    nh.param("BinningVertical", BinningVertical, 1);
    nh.param("AcquisitionFrameRate", AcquisitionFrameRate, (float)10.0);
    nh.param("AcquisitionFrameRateEnable", AcquisitionFrameRateEnable, true);
    nh.param("TriggerMode", TriggerMode, 0);
    nh.param("TriggerSource", TriggerSource, 0);
    nh.param("TriggerActivation", TriggerActivation, 0);
    nh.param("ExposureTime", ExposureTime, 40000);
    nh.param("ExposureAuto", ExposureAuto, 0);
    nh.param("AutoExposureTimeLowerLimit", AutoExposureTimeLowerLimit, 3);
    nh.param("AutoExposureTimeUpperLimit", AutoExposureTimeUpperLimit, 80000);
    nh.param("BlackLevel", BlackLevel, 100);
    nh.param("BlackLevelEnable", BlackLevelEnable, true);
    nh.param("BalanceWhiteAuto", BalanceWhiteAuto, 1);
    nh.param("Gamma", Gamma, (float)0.7);
    nh.param("GammaEnable", GammaEnable, true);

    if(BinningHorizontal == 1 && BinningVertical == 1) {
        nRet = MV_CC_SetIntValueEx(handle, "Width", Width);
        if (nRet != MV_OK) {
            ROS_ERROR("Failed to set Width! Error code: %d", nRet);
        }

        nRet = MV_CC_SetIntValueEx(handle, "Height", Height);
        if (nRet != MV_OK) {
            ROS_ERROR("Failed to set Height! Error code: %d", nRet);
        }
    } else {
        nRet = MV_CC_SetEnumValue(handle, "BinningHorizontal", BinningHorizontal);
        if (nRet != MV_OK) {
            ROS_ERROR("Failed to set BinningHorizontal! Error code: %d", nRet);
        }

        nRet = MV_CC_SetEnumValue(handle, "BinningVertical", BinningVertical);
        if (nRet != MV_OK) {
            ROS_ERROR("Failed to set BinningVertical! Error code: %d", nRet);
        }
    }

    nRet = MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", AcquisitionFrameRate);
    if (nRet != MV_OK) {
        ROS_ERROR("Failed to set AcquisitionFrameRate! Error code: %d", nRet);
    }
    
    nRet = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", AcquisitionFrameRateEnable);
    if (nRet != MV_OK) {
        ROS_ERROR("Failed to set AcquisitionFrameRateEnable! Error code: %d", nRet);
    }

    nRet = MV_CC_SetEnumValue(handle, "TriggerMode", TriggerMode);
    if (nRet != MV_OK) {
        ROS_ERROR("Failed to set TriggerMode! Error code: %d", nRet);
    }

    nRet = MV_CC_SetEnumValue(handle, "TriggerSource", TriggerSource);
    if (nRet != MV_OK) {
        ROS_ERROR("Failed to set TriggerSource! Error code: %d", nRet);
    }

    nRet = MV_CC_SetEnumValue(handle, "TriggerActivation", TriggerActivation);
    if (nRet != MV_OK) {
        ROS_ERROR("Failed to set TriggerActivation! Error code: %d", nRet);
    }

    nRet = MV_CC_SetEnumValue(handle, "ExposureAuto", ExposureAuto);
    if (nRet != MV_OK) {
        ROS_ERROR("Failed to set ExposureAuto! Error code: %d", nRet);
    }

    if(ExposureAuto == 0) {
        nRet = MV_CC_SetFloatValue(handle, "ExposureTime", ExposureTime);
        if (nRet != MV_OK) {
            ROS_ERROR("Failed to set ExposureTime! Error code: %d", nRet);
        }
    } else {
        nRet = MV_CC_SetIntValueEx(handle, "AutoExposureTimeLowerLimit", AutoExposureTimeLowerLimit);
        if (nRet != MV_OK) {
            ROS_ERROR("Failed to set AutoExposureTimeLowerLimit! Error code: %d", nRet);
        }

        nRet = MV_CC_SetIntValueEx(handle, "AutoExposureTimeUpperLimit", AutoExposureTimeUpperLimit);
        if (nRet != MV_OK) {
            ROS_ERROR("Failed to set AutoExposureTimeUpperLimit! Error code: %d", nRet);
        }
    }

    nRet = MV_CC_SetIntValueEx(handle, "BlackLevel", BlackLevel);
    if (nRet != MV_OK) {
        ROS_ERROR("Failed to set BlackLevel! Error code: %d", nRet);
    }

    nRet = MV_CC_SetBoolValue(handle, "BlackLevelEnable", BlackLevelEnable);
    if (nRet != MV_OK) {
        ROS_ERROR("Failed to set BlackLevelEnable! Error code: %d", nRet);
    }

    nRet = MV_CC_SetEnumValue(handle, "BalanceWhiteAuto", BalanceWhiteAuto);
    if (nRet != MV_OK) {
        ROS_ERROR("Failed to set BalanceWhiteAuto! Error code: %d", nRet);
    }

    nRet = MV_CC_SetFloatValue(handle, "Gamma", Gamma);
    if (nRet != MV_OK) {
        ROS_ERROR("Failed to set Gamma! Error code: %d", nRet);
    }

    nRet = MV_CC_SetBoolValue(handle, "GammaEnable", GammaEnable);
    if (nRet != MV_OK) {
        ROS_ERROR("Failed to set GammaEnable! Error code: %d", nRet);
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
