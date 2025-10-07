#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <MvCameraControl.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>

void __stdcall ImageCallbackEx2(MV_FRAME_OUT* pstFrame, void *pUser, bool bAutoFree);
void __stdcall ReconnectDevice(unsigned int nMsgType, void* pUser);

class HikCameraNode : public rclcpp::Node
{
public:
    HikCameraNode() : Node("hik_camera_node"), g_bExit(false)
    {
        // 初始化SDK
        nRet = MV_CC_Initialize();
        if (MV_OK != nRet){
            RCLCPP_ERROR(this->get_logger(), "Initialize SDK fail! nRet [0x%x]", nRet);

        }

        // 枚举设备
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE | MV_GENTL_GIGE_DEVICE | MV_GENTL_CAMERALINK_DEVICE | MV_GENTL_XOF_DEVICE | MV_GENTL_CXP_DEVICE, &stDeviceList);
        if (MV_OK != nRet){
            RCLCPP_ERROR(this->get_logger(), "Enum Devices fail! nRet [0x%x]", nRet);

        }

        if (stDeviceList.nDeviceNum <= 0){
            RCLCPP_ERROR(this->get_logger(), "Find No Devices!");
        }
        
        if (stDeviceList.pDeviceInfo[nSelectNum]->nTLayerType == MV_GIGE_DEVICE) 
        {
            memcpy(g_strSerialNumber, stDeviceList.pDeviceInfo[nSelectNum]->SpecialInfo.stGigEInfo.chSerialNumber, 
                sizeof(stDeviceList.pDeviceInfo[nSelectNum]->SpecialInfo.stGigEInfo.chSerialNumber));
        }
        else if (stDeviceList.pDeviceInfo[nSelectNum]->nTLayerType == MV_USB_DEVICE)
        {
            memcpy(g_strSerialNumber, stDeviceList.pDeviceInfo[nSelectNum]->SpecialInfo.stUsb3VInfo.chSerialNumber, 
                sizeof(stDeviceList.pDeviceInfo[nSelectNum]->SpecialInfo.stUsb3VInfo.chSerialNumber));
        }
        else if (stDeviceList.pDeviceInfo[nSelectNum]->nTLayerType  == MV_GENTL_GIGE_DEVICE)
        {
            memcpy(g_strSerialNumber, stDeviceList.pDeviceInfo[nSelectNum]->SpecialInfo.stGigEInfo.chSerialNumber, 
                sizeof(stDeviceList.pDeviceInfo[nSelectNum]->SpecialInfo.stGigEInfo.chSerialNumber));
        }
        else if (stDeviceList.pDeviceInfo[nSelectNum]->nTLayerType == MV_GENTL_CAMERALINK_DEVICE)
        {
            memcpy(g_strSerialNumber, stDeviceList.pDeviceInfo[nSelectNum]->SpecialInfo.stCMLInfo.chSerialNumber, 
                sizeof(stDeviceList.pDeviceInfo[nSelectNum]->SpecialInfo.stCMLInfo.chSerialNumber));
        }
        else if (stDeviceList.pDeviceInfo[nSelectNum]->nTLayerType == MV_GENTL_CXP_DEVICE)
        {
            memcpy(g_strSerialNumber, stDeviceList.pDeviceInfo[nSelectNum]->SpecialInfo.stCXPInfo.chSerialNumber, 
                sizeof(stDeviceList.pDeviceInfo[nSelectNum]->SpecialInfo.stCXPInfo.chSerialNumber));        
        }
        else if (stDeviceList.pDeviceInfo[nSelectNum]->nTLayerType == MV_GENTL_XOF_DEVICE)
        {
            memcpy(g_strSerialNumber, stDeviceList.pDeviceInfo[nSelectNum]->SpecialInfo.stXoFInfo.chSerialNumber, 
                sizeof(stDeviceList.pDeviceInfo[nSelectNum]->SpecialInfo.stXoFInfo.chSerialNumber));
        }
        else
        {
            printf("not support!\n");
        }

        // 创建相机句柄
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nSelectNum]);
        if (MV_OK != nRet){
            RCLCPP_ERROR(this->get_logger(), "Create Handle fail! nRet [0x%x]", nRet);
        }

        // 打开设备
        nRet = MV_CC_OpenDevice(handle);
        if (MV_OK != nRet){
            RCLCPP_ERROR(this->get_logger(), "Open Device fail! nRet [0x%x]", nRet);
        }
        g_bConnect = true;

        // 设置网络最佳包大小（仅对GigE相机有效）
        if (stDeviceList.pDeviceInfo[nSelectNum]->nTLayerType == MV_GIGE_DEVICE)
        {
            int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
            if (nPacketSize > 0)
            {
                nRet = MV_CC_SetIntValueEx(handle, "GevSCPSPacketSize", nPacketSize);
                if (nRet != MV_OK)
                {
                    RCLCPP_WARN(this->get_logger(), "Warning: Set Packet Size fail nRet [0x%x]!", nRet);
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Warning: Get Packet Size fail nRet [0x%x]!", nPacketSize);
            }
        }

        // 设置触发模式为off
        nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
        if (MV_OK != nRet)
        {
            RCLCPP_ERROR(this->get_logger(), "Set Trigger Mode fail! nRet [0x%x]", nRet);
        }

        /*---------------------------参数设置---------------------------------*/
        auto exp_desc = rcl_interfaces::msg::ParameterDescriptor{};
        exp_desc.description = "Exposure time in μs";
        auto gain_desc = rcl_interfaces::msg::ParameterDescriptor{};
        gain_desc.description = "Gain in dB";
        auto fps_desc = rcl_interfaces::msg::ParameterDescriptor{};
        fps_desc.description = "Target frame rate";
        auto fmt_desc = rcl_interfaces::msg::ParameterDescriptor{};
        fmt_desc.description = "Pixel format: 0=BayerGR8, 1=BayerRG8, 2=BayerGR12, 3=RGB8";

        this->declare_parameter("exposure_time", 5000.0, exp_desc);
        this->declare_parameter("gain",          10.0,   gain_desc);
        this->declare_parameter("frame_rate",    60.0,  fps_desc);
        this->declare_parameter("pixel_format",  3,     fmt_desc);

        // 初始化
        updateCameraParam("exposure_time", this->get_parameter("exposure_time").as_double());
        updateCameraParam("gain",          this->get_parameter("gain").as_double());
        updateCameraParam("frame_rate",    this->get_parameter("frame_rate").as_double());
        updateCameraParam("pixel_format",  this->get_parameter("pixel_format").as_int());

        param_cb_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto &p : params){
                if (p.get_name() == "exposure_time") {
                    updateCameraParam("exposure_time", p.as_double());
                } else if (p.get_name() == "gain") {
                    updateCameraParam("gain", p.as_double());
                } else if (p.get_name() == "frame_rate") {
                    updateCameraParam("frame_rate", p.as_double());
                } else if (p.get_name() == "pixel_format") {
                    updateCameraParam("pixel_format", p.as_int());
                }
            }
            return result;
        });

        /*----------------------断线重连设置-------------------------*/
        // 注册异常回调
        nRet = MV_CC_RegisterExceptionCallBack(handle, ReconnectDevice, this);
        if (MV_OK != nRet)
        {
            printf("Register ExceptionCallBack fail! nRet [%x]\n", nRet); 
        }

        // 注册抓图回调
        nRet = MV_CC_RegisterImageCallBackEx2(handle, ImageCallbackEx2, this, true);
        if (MV_OK != nRet)
        {
            printf("MV_CC_RegisterImageCallBackEx fail! nRet [%x]\n", nRet);

        }

        /*------------------------启动------------------------------*/
        // 开始取流
        nRet = MV_CC_StartGrabbing(handle);
        if (MV_OK != nRet)
        {
            RCLCPP_ERROR(this->get_logger(), "Start Grabbing fail! nRet [0x%x]", nRet);
        }

        // 创建图像发布者
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_raw", 10);

        // 创建工作线程
        work_thread_ = std::thread(&HikCameraNode::WorkThread, this);
    }

    ~HikCameraNode()
    {
        g_bExit = true;
        if (work_thread_.joinable())
        {
            work_thread_.join();
        }

        // 停止取流
        if (MV_OK != MV_CC_StopGrabbing(handle))
        {
            RCLCPP_WARN(this->get_logger(), "Stop Grabbing fail!");
        }

        // 关闭设备
        if (MV_OK != MV_CC_CloseDevice(handle))
        {
            RCLCPP_WARN(this->get_logger(), "Close Device fail!");
        }

        // 销毁句柄
        if (MV_OK != MV_CC_DestroyHandle(handle))
        {
            RCLCPP_WARN(this->get_logger(), "Destroy Handle fail!");
        }

        // 反初始化SDK
        if (MV_OK != MV_CC_Finalize())
        {
            RCLCPP_WARN(this->get_logger(), "Finalize SDK fail!");
        }
    }
    
private:
    void WorkThread()
    {
        int nRet = MV_OK;
        MV_FRAME_OUT stImageInfo = {0}; //图像信息结构体，图像数据在stImageInfo.pBufAddr
        memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT));
        MVCC_FLOATVALUE stParam = {0};
        while (!g_bExit && !reconnecting)
        {   nRet = MV_CC_GetFloatValue(handle, "ResultingFrameRate", &stParam);
            nRet = MV_CC_GetImageBuffer(handle, &stImageInfo, 1000); //最多阻塞1000ms           
            if (nRet == MV_OK)
            {
                printf("GetOneFrame, Width[%d], Height[%d], nFrameNum[%d], Real-time FPS = %.2f \n\n", 
                stImageInfo.stFrameInfo.nExtendWidth, stImageInfo.stFrameInfo.nExtendHeight, stImageInfo.stFrameInfo.nFrameNum, static_cast<double>(stParam.fCurValue));
                // 转换图像并发布
                img_transform(stImageInfo);

                MV_CC_FreeImageBuffer(handle, &stImageInfo);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "No data [0x%x]", nRet);
            }
            
        }
    }

    void img_transform(MV_FRAME_OUT& stImageInfo)
    {
        const int w = stImageInfo.stFrameInfo.nWidth;
        const int h = stImageInfo.stFrameInfo.nHeight;
        void*  data = stImageInfo.pBufAddr;

        /* 1. 根据当前格式决定 cv::Mat 类型 + ROS encoding */
        cv::Mat img;
        std::string encoding;
        switch (current_fmt_)
        {
        case PixFmt::Mono8:
            img = cv::Mat(h, w, CV_8UC1, data);
            encoding = "mono8";
            break;

        case PixFmt::BayerGR8:
            img = cv::Mat(h, w, CV_8UC1, data);
            encoding = "bayer_grbg8";
            break;

        case PixFmt::BayerRG8:
            img = cv::Mat(h, w, CV_8UC1, data);
            encoding = "bayer_rggb8";
            break;

        case PixFmt::BayerGB8:
            img = cv::Mat(h, w, CV_8UC1, data);
            encoding = "bayer_gbrg8";
            break;

        case PixFmt::BayerBG8:
            img = cv::Mat(h, w, CV_8UC1, data);
            encoding = "bayer_bggr8";
            break;

        case PixFmt::RGB8:
            img = cv::Mat(h, w, CV_8UC3, data);
            encoding = "rgb8";   // 不再转BGR
            break;

        default:
            RCLCPP_WARN(this->get_logger(), "Unknown format, publish as mono8");
            img = cv::Mat(h, w, CV_8UC1, data);
            encoding = "mono8";
        }

        /* 2. 填 ROS 消息 */
        sensor_msgs::msg::Image img_msg;
        img_msg.header.stamp = this->now();
        img_msg.header.frame_id = "camera";
        img_msg.height = img.rows;
        img_msg.width = img.cols;
        img_msg.encoding = encoding;
        img_msg.step = img.step;
        img_msg.data.assign(img.datastart, img.dataend);

        /* 3. 发布 */
        image_pub_->publish(img_msg);
    }
    
    void updateCameraParam(const std::string &name, double v)
    {
        int nRet = MV_OK;
        if (name == "exposure_time") {
            nRet = MV_CC_SetFloatValue(handle, "ExposureTime", v);
            if (nRet != MV_OK) RCLCPP_WARN(this->get_logger(), "Set ExposureTime fail! nRet [0x%x]", nRet);
            else RCLCPP_INFO(this->get_logger(), "Set ExposureTime -> %.1f μs", v);
        }
        else if (name == "gain") {
            nRet = MV_CC_SetFloatValue(handle, "Gain", v);
            if (nRet != MV_OK) RCLCPP_WARN(this->get_logger(), "Set Gain fail! nRet [0x%x]", nRet);
            else RCLCPP_INFO(this->get_logger(), "Set Gain -> %.1f dB", v);
        }
        else if (name == "frame_rate") {
            MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", true);
            nRet = MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", v);
            if (nRet != MV_OK) RCLCPP_WARN(this->get_logger(), "Set FrameRate fail! nRet [0x%x]", nRet);
            else RCLCPP_INFO(this->get_logger(), "Set FrameRate -> %.1f fps", v);
        }
    }

    void updateCameraParam(const std::string &name, int64_t v) // 重载
    {
        int nRet = MV_OK;
        if (name == "pixel_format") {
            int64_t fmt = 0;
            switch (v) {
                case 0: fmt = PixelType_Gvsp_BayerGR8; current_fmt_ = PixFmt::BayerGR8; break;
                case 1: fmt = PixelType_Gvsp_Mono8; current_fmt_ = PixFmt::Mono8; break;
                case 2: fmt = PixelType_Gvsp_BayerRG8; current_fmt_ = PixFmt::BayerRG8; break;
                case 3: fmt = PixelType_Gvsp_RGB8_Packed; current_fmt_ = PixFmt::RGB8;     break;
                default: fmt = PixelType_Gvsp_Mono8;
            }
            MV_CC_StopGrabbing(handle);
            nRet = MV_CC_SetEnumValue(handle, "PixelFormat", fmt);
            if (nRet != MV_OK) RCLCPP_WARN(this->get_logger(), "Set PixelFormat fail! nRet [0x%x]", nRet);
            else RCLCPP_INFO(this->get_logger(), "Set PixelFormat -> %ld", static_cast<long>(fmt));
            MV_CC_StartGrabbing(handle);
        }
    }

    int nRet;
    void* handle; // 句柄
    unsigned int nSelectNum = 0;
    MV_CC_DEVICE_INFO_LIST stDeviceList = {0};
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    std::thread work_thread_;
    enum class PixFmt : int {
        Mono8,
        BayerGR8,
        BayerRG8,
        BayerGB8,
        BayerBG8,
        RGB8,
        Unknown
    };
    PixFmt current_fmt_ = PixFmt::BayerGR8;

    // 用于设置参数
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

    // 用于断线重连的设置
    bool reconnecting = false;
    bool  g_bConnect = false;
    char  g_strSerialNumber[64] = {0};  // 相机序列号
    bool  g_bExit = false;

    friend void __stdcall ImageCallbackEx2(MV_FRAME_OUT* pstFrame, void *pUser, bool bAutoFree);
    friend void __stdcall ReconnectDevice(unsigned int nMsgType, void* pUser);
};

void __stdcall ImageCallbackEx2(MV_FRAME_OUT* pstFrame, void *pUser, bool bAutoFree)
{
    HikCameraNode* node = static_cast<HikCameraNode*>(pUser);
    if (pstFrame)
    {
        int nRet;
        MVCC_FLOATVALUE stParam = {0};
        nRet = MV_CC_GetFloatValue(node->handle, "ResultingFrameRate", &stParam);

        printf("connected, GetOneFrame, Width[%d], Height[%d], nFrameNum[%d], Real-time FPS = %.2f \n\n", 
            pstFrame->stFrameInfo.nExtendWidth, pstFrame->stFrameInfo.nExtendHeight, pstFrame->stFrameInfo.nFrameNum, static_cast<double>(stParam.fCurValue));
        
         /* 2. 根据当前格式构造 cv::Mat + encoding */
        const int w = pstFrame->stFrameInfo.nWidth;
        const int h = pstFrame->stFrameInfo.nHeight;
        void*   data = pstFrame->pBufAddr;

        cv::Mat img;
        std::string encoding;

        switch (node->current_fmt_) {   // 使用类成员变量
        case HikCameraNode::PixFmt::Mono8:
            img = cv::Mat(h, w, CV_8UC1, data);
            encoding = "mono8";
            break;
        case HikCameraNode::PixFmt::BayerGR8:
            img = cv::Mat(h, w, CV_8UC1, data);
            encoding = "bayer_grbg8";
            break;
        case HikCameraNode::PixFmt::BayerRG8:
            img = cv::Mat(h, w, CV_8UC1, data);
            encoding = "bayer_rggb8";
            break;
        case HikCameraNode::PixFmt::BayerGB8:
            img = cv::Mat(h, w, CV_8UC1, data);
            encoding = "bayer_gbrg8";
            break;
        case HikCameraNode::PixFmt::BayerBG8:
            img = cv::Mat(h, w, CV_8UC1, data);
            encoding = "bayer_bggr8";
            break;
        case HikCameraNode::PixFmt::RGB8:
            img = cv::Mat(h, w, CV_8UC3, data);
            encoding = "rgb8";
            break;
        default:
            img = cv::Mat(h, w, CV_8UC1, data);
            encoding = "mono8";
            break;
        }

        /* 3. 填 ROS 消息 */
        sensor_msgs::msg::Image img_msg;
        img_msg.header.stamp    = node->now();
        img_msg.header.frame_id = "camera";
        img_msg.height          = img.rows;
        img_msg.width           = img.cols;
        img_msg.encoding        = encoding;
        img_msg.step            = img.step;
        img_msg.data.assign(img.datastart, img.dataend);

        /* 4. 发布 */
        node->image_pub_->publish(img_msg);

        /* 5. 手动释放（当 bAutoFree==false 时） */
        if (!bAutoFree){
            MV_CC_FreeImageBuffer(node->handle, pstFrame);}
    }
}


void __stdcall ReconnectDevice(unsigned int nMsgType, void* pUser)
{
    int nRet = MV_OK;
    HikCameraNode* node = static_cast<HikCameraNode*>(pUser);
    node->reconnecting = true;
    if(nMsgType == MV_EXCEPTION_DEV_DISCONNECT)
    {
        
        if(true == node->g_bConnect)
        {
           
            nRet = MV_CC_CloseDevice(node->handle);
            nRet = MV_CC_DestroyHandle(node->handle);
            node->handle = NULL;
                
            MV_CC_DEVICE_INFO_LIST stDevTempList = { 0 };
            memset(&stDevTempList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
            
            unsigned int nIndex = -1;
            
            printf("device diconnect, please wait...\n");
            do 
            {
                int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE | MV_GENTL_GIGE_DEVICE|MV_GENTL_CAMERALINK_DEVICE | MV_GENTL_XOF_DEVICE | MV_GENTL_CXP_DEVICE , &stDevTempList);
                if (MV_OK != nRet)
                {
                    printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
                    continue;
                }
                
                bool bFind = false;
                for (unsigned int i = 0; i< stDevTempList.nDeviceNum; i++)
                {
                
                    if (stDevTempList.pDeviceInfo[i]->nTLayerType == MV_USB_DEVICE)
                    {
                        if (0 == strcmp((char*)(stDevTempList.pDeviceInfo[i]->SpecialInfo.stUsb3VInfo.chSerialNumber), node->g_strSerialNumber))
                        {
                            nIndex = i;
                            bFind = true;
                            break;
                        }
                        
                    }
                    else if ((stDevTempList.pDeviceInfo[i]->nTLayerType == MV_GIGE_DEVICE)||(stDevTempList.pDeviceInfo[i]->nTLayerType == MV_GENTL_GIGE_DEVICE))
                    {
                        if (0 == strcmp((char*)(stDevTempList.pDeviceInfo[i]->SpecialInfo.stGigEInfo.chSerialNumber), node->g_strSerialNumber))
                        {
                            nIndex = i;
                            bFind = true;
                            break;
                        }
                        
                    }
                    else if (stDevTempList.pDeviceInfo[i]->nTLayerType == MV_GENTL_CAMERALINK_DEVICE)
                    {
                        if (0 == strcmp((char*)(stDevTempList.pDeviceInfo[i]->SpecialInfo.stCMLInfo.chSerialNumber), node->g_strSerialNumber))
                        {
                            nIndex = i;
                            bFind = true;
                            break;
                        }
                    }
                    else if (stDevTempList.pDeviceInfo[i]->nTLayerType == MV_GENTL_CXP_DEVICE)
                    {
                        if (0 == strcmp((char*)(stDevTempList.pDeviceInfo[i]->SpecialInfo.stCXPInfo.chSerialNumber), node->g_strSerialNumber))
                        {
                            nIndex = i;
                            bFind = true;
                            break;
                        }
                    }
                    else if (stDevTempList.pDeviceInfo[i]->nTLayerType == MV_GENTL_XOF_DEVICE)
                    {
                        if (0 == strcmp((char*)(stDevTempList.pDeviceInfo[i]->SpecialInfo.stXoFInfo.chSerialNumber), node->g_strSerialNumber))
                        {
                            nIndex = i;
                            bFind = true;
                            break;
                        }
                    }

                }
                
                if ((-1 == (int)nIndex) || (false == bFind))
                {
                    continue;
                }
                
                // 选择设备并创建句柄
                // select device and create handle
                nRet = MV_CC_CreateHandle(&node->handle, stDevTempList.pDeviceInfo[nIndex]);
                if (MV_OK != nRet)
                {
                    printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
                    continue;
                }

                // 打开设备
                // open device
                nRet = MV_CC_OpenDevice(node->handle);
                if (MV_OK != nRet)
                {
                    printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
                    MV_CC_DestroyHandle(node->handle);
                    node->handle = NULL;
                    continue;
                }
                else
                {
                    node->reconnecting = false;
                    break;  // open success退出循环
                }
                
            }while(node->g_bExit== false);

            
            MV_CC_RegisterExceptionCallBack(node->handle, ReconnectDevice, node);

            // register image callback
            MV_CC_RegisterImageCallBackEx2(node->handle, ImageCallbackEx2, node, true);
            
            nRet = MV_CC_StartGrabbing(node->handle);
            if (MV_OK != nRet)
            {
                printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
                return;
            }
            

        }
    }
    
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HikCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}