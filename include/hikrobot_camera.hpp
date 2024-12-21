#ifndef CAMERA_HPP
#define CAMERA_HPP
#include "ros/ros.h"
#include <stdio.h>
#include <thread>
#include <opencv2/opencv.hpp>
#include "MvErrorDefine.h"
#include "CameraParams.h"
#include "MvCameraControl.h"

#define MAX_IMAGE_DATA_SIZE (4 * 2048 * 3072)
namespace camera
{
    enum CamerProperties
    {
        CAP_PROP_FRAMERATE_ENABLE,  //帧数可调
        CAP_PROP_FRAMERATE,         //帧数
        CAP_PROP_BURSTFRAMECOUNT,   //外部一次触发帧数
        CAP_PROP_HEIGHT,            //图像高度
        CAP_PROP_WIDTH,             //图像宽度
        CAP_PROP_EXPOSURE_TIME,     //曝光时间
        CAP_PROP_GAMMA_ENABLE,      //伽马因子可调
        CAP_PROP_GAMMA,             //伽马因子
        CAP_PROP_GAINAUTO,          //亮度
        CAP_PROP_SATURATION_ENABLE, //饱和度可调
        CAP_PROP_SATURATION,        //饱和度
        CAP_PROP_OFFSETX,           //X偏置
        CAP_PROP_OFFSETY,           //Y偏置
        CAP_PROP_TRIGGER_MODE,      //外部触发
        CAP_PROP_TRIGGER_SOURCE,    //触发源
        CAP_PROP_LINE_SELECTOR      //触发线
    };
    
    class Camera
    {
    public:
        /**
         * @brief 构造函数，初始化 Camera 实例。
         * @param node ROS 节点句柄。
         */
        Camera(ros::NodeHandle &node, std::string topic_name);

        /**
         * @brief 析构函数，释放 Camera 相关资源。
         */
        ~Camera();

        /**
         * @brief 工作函数。
         * @return 返回处理的结果。
         */
        void *HKRun();

        /**
         * @brief 打印摄像头设备信息。
         * @param pstMVDevInfo 指向设备信息结构体的指针。
         * @return 是否成功打印设备信息。
         */
        bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo);

        /**
         * @brief 设置摄像头参数。
         * @param type 参数类型（如曝光时间、增益等）。
         * @param value 参数值。
         * @return 是否成功设置参数。
         */
        bool set(camera::CamerProperties type, float value);

        /**
         * @brief 恢复摄像头默认参数。
         * @return 是否成功恢复默认参数。
         */
        bool reset();

        /**
         * @brief 从摄像头读取原始图像。
         * @param image OpenCV 的 Mat 对象，用于存储读取的图像。
         */
        void ReadImg(cv::Mat &image);

    private:
        void *handle;             // 摄像头句柄，用于与硬件交互。
        pthread_t nThreadID;      // 线程 ID，用于管理摄像头处理线程。
        int nRet;                 // 返回值变量，用于存储函数返回状态。

        int width;                // 图像宽度配置。
        int height;               // 图像高度配置。
        int Offset_x;             // 图像偏移量（X 轴）。
        int Offset_y;             // 图像偏移量（Y 轴）。
        bool FrameRateEnable;     // 帧率启用开关。
        int FrameRate;            // 配置的帧率值。
        int BurstFrameCount;      // 连拍帧数设置。
        int ExposureTime;         // 曝光时间设置（单位：微秒）。
        bool GammaEnable;         // 伽马校正启用开关。
        float Gamma;              // 伽马值配置。
        int GainAuto;             // 增益自动调整模式。
        bool SaturationEnable;    // 饱和度调整启用开关。
        int Saturation;           // 饱和度值配置。
        int TriggerMode;          // 触发模式配置。
        int TriggerSource;        // 触发源配置。
        int LineSelector;         // 线路选择配置。

        image_transport::ImageTransport main_cam_image;
        image_transport::CameraPublisher image_pub;
    };

    /**
     * @brief 构造函数，初始化 Camera 实例。
     * @param node ROS 节点句柄。
     */
    Camera::Camera(ros::NodeHandle &node, std::string topic_name) : main_cam_image(node)
    {
        handle = NULL;

        image_pub = main_cam_image.advertiseCamera(topic_name, 1000);
        // 读取摄像头参数配置，如果未提供，则使用默认值
        node.param("width", width, 2600);                      // 图像宽度
        node.param("height", height, 2160);                    // 图像高度
        node.param("FrameRateEnable", FrameRateEnable, false); // 帧率启用开关
        node.param("FrameRate", FrameRate, 10);                // 帧率值
        node.param("BurstFrameCount", BurstFrameCount, 10);    // 一次触发采集的帧数
        node.param("ExposureTime", ExposureTime, 50000);       // 曝光时间（单位：微秒）
        node.param("GammaEnable", GammaEnable, false);         // 伽马校正启用开关
        node.param("Gamma", Gamma, (float)0.7);                // 伽马值
        node.param("GainAuto", GainAuto, 2);                   // 自动增益模式
        node.param("SaturationEnable", SaturationEnable, true);// 饱和度调整启用开关
        node.param("Saturation", Saturation, 128);             // 饱和度值
        node.param("Offset_x", Offset_x, 0);                   // 图像偏移量（X 轴）
        node.param("Offset_y", Offset_y, 0);                   // 图像偏移量（Y 轴）
        node.param("TriggerMode", TriggerMode, 1);             // 触发模式
        node.param("TriggerSource", TriggerSource, 2);         // 触发源
        node.param("LineSelector", LineSelector, 2);           // 线路选择

        // 枚举设备
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
            printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
            exit(-1);
        }

        // 打印设备信息
        if (stDeviceList.nDeviceNum > 0)
        {
            for (int i = 0; i < stDeviceList.nDeviceNum; i++)
            {
                printf("[device %d]:\n", i);
                MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
                if (NULL == pDeviceInfo)
                {
                    break;
                }
                PrintDeviceInfo(pDeviceInfo); // 打印当前设备信息
            }
        }
        else
        {
            printf("Find No Devices!\n");
            exit(-1);
        }

        // 选择设备并创建句柄
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[0]);
        if (MV_OK != nRet)
        {
            printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
            exit(-1);
        }

        // 打开设备
        nRet = MV_CC_OpenDevice(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
            exit(-1);
        }

        //设置摄像头参数
        this->set(CAP_PROP_FRAMERATE_ENABLE, FrameRateEnable); // 启用帧率控制
        if (FrameRateEnable) 
            this->set(CAP_PROP_FRAMERATE, FrameRate);         // 设置帧率值
        this->set(CAP_PROP_BURSTFRAMECOUNT, BurstFrameCount); // 设置连拍帧数
        this->set(CAP_PROP_HEIGHT, height);                  // 设置图像高度
        this->set(CAP_PROP_WIDTH, width);                    // 设置图像宽度
        this->set(CAP_PROP_OFFSETX, Offset_x);               // 设置图像偏移量（X 轴）
        this->set(CAP_PROP_OFFSETY, Offset_y);               // 设置图像偏移量（Y 轴）
        this->set(CAP_PROP_EXPOSURE_TIME, ExposureTime);      // 设置曝光时间
        this->set(CAP_PROP_GAMMA_ENABLE, GammaEnable);       // 启用伽马校正
        if (GammaEnable) 
            this->set(CAP_PROP_GAMMA, Gamma);                // 设置伽马值
        this->set(CAP_PROP_GAINAUTO, GainAuto);              // 设置自动增益模式
        this->set(CAP_PROP_TRIGGER_MODE, TriggerMode);       // 设置触发模式
        this->set(CAP_PROP_TRIGGER_SOURCE, TriggerSource);   // 设置触发源
        
        this->set(CAP_PROP_SATURATION_ENABLE, SaturationEnable); // 设置饱和度
        if (SaturationEnable) 
            this->set(CAP_PROP_SATURATION, Saturation);
        // this->set(CAP_PROP_LINE_SELECTOR, LineSelector);

        // 设置白平衡
        nRet = MV_CC_SetEnumValue(handle, "BalanceWhiteAuto", 0);
        // 白平衡度
        // int rgb[3] = {1742, 1024, 2371};
        // for (int i = 0; i < 3; i++)
        // {
        //     nRet = MV_CC_SetEnumValue(handle, "BalanceRatioSelector", i);
        //     nRet = MV_CC_SetIntValue(handle, "BalanceRatio", rgb[i]);
        // }
        if (MV_OK == nRet)
        {
            printf("set BalanceRatio OK! value=%f\n",0.0 );
        }
        else
        {
            printf("Set BalanceRatio Failed! nRet = [%x]\n\n", nRet);
        }


        // 设置图像格式为 RGB
        // 0x01100003:Mono10
        // 0x010C0004:Mono10Packed
        // 0x01100005:Mono12
        // 0x010C0006:Mono12Packed
        // 0x01100007:Mono16
        // 0x02180014:RGB8Packed
        // 0x02100032:YUV422_8
        // 0x0210001F:YUV422_8_UYVY
        // 0x01080008:BayerGR8
        // 0x01080009:BayerRG8
        // 0x0108000A:BayerGB8
        // 0x0108000B:BayerBG8
        // 0x0110000e:BayerGB10
        // 0x01100012:BayerGB12
        // 0x010C002C:BayerGB12Packed
        nRet = MV_CC_SetEnumValue(handle, "PixelFormat", 0x02180014);
        if (MV_OK == nRet)
        {
            printf("set PixelFormat OK ! value = RGB\n");
        }
        else
        {
            printf("MV_CC_SetPixelFormat fail! nRet [%x]\n", nRet);
        }

        // 检查图像格式
        MVCC_ENUMVALUE t = {0};
        nRet = MV_CC_GetEnumValue(handle, "PixelFormat", &t);
        if (MV_OK == nRet)
        {
            printf("PixelFormat :%d!\n", t.nCurValue);
        }
        else
        {
            printf("get PixelFormat fail! nRet [%x]\n", nRet);
        }

        // 开始取流
        nRet = MV_CC_StartGrabbing(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
            exit(-1);
        }
    }

    /**
     * @brief 析构函数，释放 Camera 相关资源。
     */
    Camera::~Camera()
    {
        int nRet;
        
        // 等待线程结束
        pthread_join(nThreadID, NULL);

        // 停止取流
        nRet = MV_CC_StopGrabbing(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
            exit(-1);
        }
        printf("MV_CC_StopGrabbing succeed.\n");
        
        // 关闭设备
        nRet = MV_CC_CloseDevice(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
            exit(-1);
        }
        printf("MV_CC_CloseDevice succeed.\n");
        
        // 销毁句柄
        nRet = MV_CC_DestroyHandle(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
            exit(-1);
        }
        printf("MV_CC_DestroyHandle succeed.\n");
        
        // 销毁互斥量
        pthread_mutex_destroy(&mutex);
    }

    /**
     * @brief 设置摄像头的指定参数。
     *
     * @param type 参数类型（枚举值），用于指定要设置的摄像头属性。
     * @param value 参数值，设置为具体的数值或布尔值。
     * @return 成功返回非零值，失败返回 0。
     */
    bool Camera::set(CamerProperties type, float value)
    {
        switch (type)
        {
        case CAP_PROP_FRAMERATE_ENABLE:
        {
            // 设置帧率启用开关
            nRet = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", value);
            if (MV_OK == nRet)
            {
                printf("set AcquisitionFrameRateEnable OK! value=%f\n",value);
            }
            else
            {
                printf("Set AcquisitionFrameRateEnable Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_FRAMERATE:
        {
            // 设置帧率值
            nRet = MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", value);
            if (MV_OK == nRet)
            {
                printf("set AcquisitionFrameRate OK! value=%f\n",value);
            }
            else
            {
                printf("Set AcquisitionFrameRate Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_BURSTFRAMECOUNT:
        {
            // 设置连拍帧数
            nRet = MV_CC_SetIntValue(handle, "AcquisitionBurstFrameCount", value);
            if (MV_OK == nRet)
            {
                printf("set AcquisitionBurstFrameCount OK!\n");
            }
            else
            {
                printf("Set AcquisitionBurstFrameCount Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_HEIGHT:
        {
            // 设置图像高度
            nRet = MV_CC_SetIntValue(handle, "Height", value);

            if (MV_OK == nRet)
            {
                printf("set Height OK!\n");
            }
            else
            {
                printf("Set Height Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_WIDTH:
        {
            // 设置图像宽度
            nRet = MV_CC_SetIntValue(handle, "Width", value);
            if (MV_OK == nRet)
            {
                printf("set Width OK!\n");
            }
            else
            {
                printf("Set Width Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_OFFSETX:
        {
            // 设置图像偏移量（X 轴）
            nRet = MV_CC_SetIntValue(handle, "OffsetX", value);
            if (MV_OK == nRet)
            {
                printf("set Offset X OK!\n");
            }
            else
            {
                printf("Set Offset X Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_OFFSETY:
        {
            // 设置图像偏移量（Y 轴）
            nRet = MV_CC_SetIntValue(handle, "OffsetY", value);
            if (MV_OK == nRet)
            {
                printf("set Offset Y OK!\n");
            }
            else
            {
                printf("Set Offset Y Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_EXPOSURE_TIME:
        {
            // 设置曝光时间
            nRet = MV_CC_SetFloatValue(handle, "ExposureTime", value);
            if (MV_OK == nRet)
            {
                printf("set ExposureTime OK! value=%f\n",value);
            }
            else
            {
                printf("Set ExposureTime Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_GAMMA_ENABLE:
        {
            // 设置伽马启用开关
            nRet = MV_CC_SetBoolValue(handle, "GammaEnable", value);
            if (MV_OK == nRet)
            {
                printf("set GammaEnable OK! value=%f\n",value);
            }
            else
            {
                printf("Set GammaEnable Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_GAMMA:
        {
            // 设置伽马值
            nRet = MV_CC_SetFloatValue(handle, "Gamma", value);
            if (MV_OK == nRet)
            {
                printf("set Gamma OK! value=%f\n",value);
            }
            else
            {
                printf("Set Gamma Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_GAINAUTO:
        {
            // 设置自动增益模式
            nRet = MV_CC_SetEnumValue(handle, "GainAuto", value);
            if (MV_OK == nRet)
            {
                printf("set GainAuto OK! value=%f\n",value);
            }
            else
            {
                printf("Set GainAuto Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_SATURATION_ENABLE:
        {
            // 设置饱和度启用开关
            nRet = MV_CC_SetBoolValue(handle, "SaturationEnable", value);
            if (MV_OK == nRet)
            {
                printf("set SaturationEnable OK! value=%f\n",value);
            }
            else
            {
                printf("Set SaturationEnable Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_SATURATION:
        {
            // 设置饱和度值
            nRet = MV_CC_SetIntValue(handle, "Saturation", value);
            if (MV_OK == nRet)
            {
                printf("set Saturation OK! value=%f\n",value);
            }
            else
            {
                printf("Set Saturation Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }

        case CAP_PROP_TRIGGER_MODE:
        {
            // 设置触发模式
            nRet = MV_CC_SetEnumValue(handle, "TriggerMode", value);
            if (MV_OK == nRet)
            {
                printf("set TriggerMode OK!\n");
            }
            else
            {
                printf("Set TriggerMode Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_TRIGGER_SOURCE:
        {
            // 设置触发源
            nRet = MV_CC_SetEnumValue(handle, "TriggerSource", value);
            if (MV_OK == nRet)
            {
                printf("set TriggerSource OK!\n");
            }
            else
            {
                printf("Set TriggerSource Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_LINE_SELECTOR:
        {
            // 设置线路选择
            nRet = MV_CC_SetEnumValue(handle, "LineSelector", value);
            if (MV_OK == nRet)
            {
                printf("set LineSelector OK!\n");
            }
            else
            {
                printf("Set LineSelector Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }

        default:
            return 0;
        }
        return nRet;
    }

    /**
     * @brief 恢复摄像头默认参数。
     * @return 是否成功恢复默认参数。
     */
    bool Camera::reset()
    {
        nRet = this->set(CAP_PROP_FRAMERATE_ENABLE, FrameRateEnable);
        nRet = this->set(CAP_PROP_FRAMERATE, FrameRate) || nRet;
        // nRet = this->set(CAP_PROP_BURSTFRAMECOUNT, BurstFrameCount) || nRet;
        nRet = this->set(CAP_PROP_HEIGHT, height) || nRet;
        nRet = this->set(CAP_PROP_WIDTH, width) || nRet;
        nRet = this->set(CAP_PROP_OFFSETX, Offset_x) || nRet;
        nRet = this->set(CAP_PROP_OFFSETY, Offset_y) || nRet;
        nRet = this->set(CAP_PROP_EXPOSURE_TIME, ExposureTime) || nRet;
        nRet = this->set(CAP_PROP_GAMMA_ENABLE, GammaEnable) || nRet;
        nRet = this->set(CAP_PROP_GAMMA, Gamma) || nRet;
        nRet = this->set(CAP_PROP_GAINAUTO, GainAuto) || nRet;
        nRet = this->set(CAP_PROP_SATURATION_ENABLE, SaturationEnable) || nRet;
        nRet = this->set(CAP_PROP_SATURATION, Saturation) || nRet;
        nRet = this->set(CAP_PROP_TRIGGER_MODE, TriggerMode) || nRet;
        nRet = this->set(CAP_PROP_TRIGGER_SOURCE, TriggerSource) || nRet;
        nRet = this->set(CAP_PROP_LINE_SELECTOR, LineSelector) || nRet;
        return nRet;
    }

    /**
     * @brief 打印摄像头设备信息。
     * @param pstMVDevInfo 指向设备信息结构体的指针。
     * @return 是否成功打印设备信息。
     */
    bool Camera::PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo)
    {
        if (NULL == pstMVDevInfo)
        {
            printf("%s\n", "The Pointer of pstMVDevInfoList is NULL!");
            return false;
        }
        if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
        {
            printf("%s %x\n", "nCurrentIp:", pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp);                 //当前IP
            printf("%s %s\n\n", "chUserDefinedName:", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName); //用户定义名
        }
        else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
        {
            printf("UserDefinedName:%s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
        }
        else
        {
            printf("Not support.\n");
        }
        return true;
    }

    /**
     * @brief 工作函数。
     * @return 返回处理的结果。
     */
    void *Camera::HKRun()
    {
        int nRet;
        // double start; // 记录帧采集开始时间
        unsigned char *m_pBufForDriver = (unsigned char *)malloc(sizeof(unsigned char) * MAX_IMAGE_DATA_SIZE); // 驱动层数据缓存
        unsigned char *m_pBufForSaveImage = (unsigned char *)malloc(MAX_IMAGE_DATA_SIZE); // 转换后图像数据缓存
        MV_FRAME_OUT_INFO_EX stImageInfo = {0}; // 图像帧信息
        MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0}; // 像素格式转换参数
        int image_empty_count = 0; // 记录连续读取失败的空帧计数
        
        sensor_msgs::Image image_msg;
        sensor_msgs::CameraInfo camera_info_msg;
        cv_bridge::CvImagePtr cv_ptr = boost::make_shared<cv_bridge::CvImage>();
        cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;

        while (ros::ok())
        {
            // start = static_cast<double>(cv::getTickCount()); // 获取当前时间戳

            // 尝试从相机读取一帧图像数据，设置超时时间为 100ms
            nRet = MV_CC_GetOneFrameTimeout(p_handle, m_pBufForDriver, MAX_IMAGE_DATA_SIZE, &stImageInfo, 100);
            if (nRet != MV_OK)
            {
                if (++image_empty_count > 10) // 如果读取失败，递增空帧计数器，当计数器超过阈值时退出程序
                {
                    ROS_ERROR("The Number of Faild Reading Exceed The Set Value!\n");
                }
                continue;
            }
            image_empty_count = 0; // 成功读取帧后重置空帧计数器
            
            stConvertParam.nWidth = width;                               //ch:图像宽 | en:image width
            stConvertParam.nHeight = height;                             //ch:图像高 | en:image height

            stConvertParam.pSrcData = m_pBufForDriver;                  //ch:输入数据缓存 | en:input data buffer
            stConvertParam.nSrcDataLen = MAX_IMAGE_DATA_SIZE;           //ch:输入数据大小 | en:input data size
            stConvertParam.enSrcPixelType = stImageInfo.enPixelType;    //ch:输入像素格式 | en:input pixel format

            stConvertParam.pDstBuffer = m_pBufForSaveImage;             //ch:输出数据缓存 | en:output data buffer
            stConvertParam.nDstBufferSize = MAX_IMAGE_DATA_SIZE;        //ch:输出缓存大小 | en:output buffer size
            stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed; //ch:输出像素格式 | en:output pixel format
            MV_CC_ConvertPixelType(p_handle, &stConvertParam); // 转换图像格式为BGR8
            
            cv::Mat frame = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, m_pBufForSaveImage).clone();

            cv_ptr->image = frame;
            image_msg = *(cv_ptr->toImageMsg());
            image_msg.header.stamp = ros::Time::now();
            image_msg.header.frame_id = "hikrobot_camera";

            camera_info_msg.header.frame_id = image_msg.header.frame_id;
            camera_info_msg.header.stamp = image_msg.header.stamp;
            image_pub.publish(image_msg, camera_info_msg);
            
            //*************************************testing img********************************//
            // double time = ((double)cv::getTickCount() - start) / cv::getTickFrequency();
            // std::cout << "HK_camera,Time:" << time << "\tFPS:" << 1 / time << std::endl;
            // imshow("HK vision",frame);
            // waitKey(1);
        }

        // 释放内存资源
        free(m_pBufForDriver);
        free(m_pBufForSaveImage);

        return 0;
    }

} // namespace camera
#endif
