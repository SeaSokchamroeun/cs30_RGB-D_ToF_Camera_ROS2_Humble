#include "synexens_ros2/SYRosDevice.h"

SYRosDevice::SYRosDevice(rclcpp::Node::SharedPtr nd) : m_node(nd)
{
    // set params
    m_parmas.SetParams(m_node);
    // get params
    m_cameraParams = m_parmas.GetParams();
    // get coamer config
    // m_parmas.GetCameraConfig(&m_cameraConfig);
}

SYRosDevice::~SYRosDevice()
{
    stopCameras();
}

void SYRosDevice::stopCameras()
{
    for (int i = 0; i < m_nCount; i++)
    {
        if (m_pOpen[i])
            Synexens::StopStreaming(m_pDeviceInfo[i].m_nDeviceID);
    }
    delete[] m_pDeviceInfo;
    m_pDeviceInfo = nullptr;
    delete[] m_pOpen;
    m_pOpen = nullptr;

    Synexens::UnInitSDK();
}

void SYRosDevice::OnFrameNotify(unsigned int nDeviceID, Synexens::SYFrameData *pFrameData)
{
    ProcessFrameData(nDeviceID, pFrameData);
}

void SYRosDevice::PrintErrorCode(std::string strFunc, Synexens::SYErrorCode errorCode)
{
    RCLCPP_ERROR(m_node->get_logger(), "%s errorcode:%d", strFunc.c_str(), errorCode);
}

SYErrorCode SYRosDevice::startCameras()
{
    // ============= SDK init start ============= //
    int nSDKVersionLength = 0;
    Synexens::SYErrorCode errorCodeGetSDKVersion = Synexens::GetSDKVersion(nSDKVersionLength, nullptr);
    if (errorCodeGetSDKVersion == Synexens::SYERRORCODE_SUCCESS)
    {
        if (nSDKVersionLength > 0)
        {
            char *pStringSDKVersion = new char[nSDKVersionLength];
            errorCodeGetSDKVersion = Synexens::GetSDKVersion(nSDKVersionLength, pStringSDKVersion);
            if (errorCodeGetSDKVersion == Synexens::SYERRORCODE_SUCCESS)
            {
                RCLCPP_INFO(m_node->get_logger(), "SDKVersion:%s", pStringSDKVersion);
            }
            else
            {
                PrintErrorCode("GetSDKVersion2", errorCodeGetSDKVersion);
                return errorCodeGetSDKVersion;
            }
            delete[] pStringSDKVersion;
        }
    }
    else
    {
        PrintErrorCode("GetSDKVersion", errorCodeGetSDKVersion);
    }

    Synexens::SYErrorCode errorCodeInitSDK = Synexens::InitSDK();
    if (errorCodeInitSDK != Synexens::SYERRORCODE_SUCCESS)
    {
        PrintErrorCode("InitSDK", errorCodeInitSDK);
        return errorCodeInitSDK;
    }
    m_calibrationData.initialize(m_cameraParams, m_node);
    // ============= SDK init end ============= //

    // ============= openDevice and openFlow ============= //
    int nCount = 0;
    Synexens::SYErrorCode errorCode = Synexens::FindDevice(nCount);
    if (errorCode == Synexens::SYERRORCODE_SUCCESS && nCount > 0)
    {
        m_pDeviceInfo = new Synexens::SYDeviceInfo[nCount];
        errorCode = Synexens::FindDevice(nCount, m_pDeviceInfo);

        m_nCount = nCount;
        if (errorCode == Synexens::SYERRORCODE_SUCCESS)
        {
            m_pOpen = new bool[nCount];
            memset(m_pOpen, 0, sizeof(bool) * nCount);
            for (int i = 0; i < nCount; i++)
            {
                Synexens::SYErrorCode errorCodeOpenDevice = Synexens::OpenDevice(m_pDeviceInfo[i]);
                if (errorCodeOpenDevice == Synexens::SYERRORCODE_SUCCESS)
                {
                    // get sn
                    int nStringLength = 0;
                    Synexens::SYErrorCode errorCodeGetSN = Synexens::GetDeviceSN(m_pDeviceInfo[i].m_nDeviceID, nStringLength, nullptr);
                    if (errorCodeGetSN == Synexens::SYERRORCODE_SUCCESS)
                    {
                        if (nStringLength > 0)
                        {
                            char *pStringSN = new char[nStringLength];
                            errorCodeGetSN = Synexens::GetDeviceSN(m_pDeviceInfo[i].m_nDeviceID, nStringLength, pStringSN);
                            if (errorCodeGetSN == Synexens::SYERRORCODE_SUCCESS)
                            {
                                RCLCPP_INFO(m_node->get_logger(), "SN%d:%s\n", i, pStringSN);
                                std::string deviceSN = pStringSN;
                                SYRosDeviceInfo rosDeviceInfo;
                                rosDeviceInfo.m_deviceType = m_pDeviceInfo[i].m_deviceType;
                                rosDeviceInfo.m_deviceSN = deviceSN;
                                m_mapRosDeviceInfo.insert(std::pair<int, SYRosDeviceInfo>(m_pDeviceInfo[i].m_nDeviceID, rosDeviceInfo));
                            }
                            else
                            {
                                PrintErrorCode("GetDeviceSN", errorCodeGetSN);
                            }
                            delete[] pStringSN;
                        }
                    }
                    else
                    {
                        PrintErrorCode("GetDeviceSN", errorCodeGetSN);
                    }
                }
                else
                {
                    PrintErrorCode("OpenDevice", errorCodeOpenDevice);
                }
            }
            // get HWVersion
            GetHWVersion();
            // register topics
            RegisterTopic();
            // Import configuration and open stream
            ImportConfigurationStream();

            RCLCPP_INFO(m_node->get_logger(), "Synexens Started");
            rclcpp::WallRate loop_rate(60);
            while (rclcpp::ok())
            {
                for (int nDeviceIndex = 0; nDeviceIndex < nCount; nDeviceIndex++)
                {
                    if (m_pOpen[nDeviceIndex])
                    {
                        // frame data
                        Synexens::SYFrameData *pLastFrameData = nullptr;
                        Synexens::SYErrorCode errorCodeLastFrame = Synexens::GetLastFrameData(m_pDeviceInfo[nDeviceIndex].m_nDeviceID, pLastFrameData);
                        if (errorCodeLastFrame == Synexens::SYERRORCODE_SUCCESS)
                        {
                            ProcessFrameData(m_pDeviceInfo[nDeviceIndex].m_nDeviceID, pLastFrameData);
                        }
                    }
                }

                rclcpp::spin_some(m_node);
                loop_rate.sleep();
            }
        }
        else
        {
            PrintErrorCode("FindDevice2", errorCode);
            return errorCode;
        }
    }
    else
    {
        PrintErrorCode("FindDevice1", errorCode);
        return Synexens::SYERRORCODE_FAILED;
    }

    errorCodeInitSDK = Synexens::UnInitSDK();
    if (errorCodeInitSDK != Synexens::SYERRORCODE_SUCCESS)
    {
        PrintErrorCode("UnInitSDK", errorCodeInitSDK);
    }
    return Synexens::SYERRORCODE_SUCCESS;
}

void SYRosDevice::RegisterTopic()
{
    RCLCPP_INFO(m_node->get_logger(), "RegisterTopic in");
    // init m_imageTranspoort
    image_transport::ImageTransport m_imageTranspoort(m_node);

    for (int i = 0; i < m_nCount; i++)
    {
        // ============= register topic ============= //
        std::map<PUBLISHER_TYPE, rclcpp::Publisher<CameraInfo>::SharedPtr> mapRosPublisher;
        std::map<PUBLISHER_TYPE, image_transport::Publisher> mapImagePublisher;
        std::map<PUBLISHER_TYPE, rclcpp::Publisher<PointCloud2>::SharedPtr> mapPointsPublisher;
        int nDeviceID = m_pDeviceInfo[i].m_nDeviceID;
        // ros pulisher
        // depth
        std::string sTopicTopName = "camera" + std::to_string(m_pDeviceInfo[i].m_nDeviceID) + "_" + m_mapRosDeviceInfo[nDeviceID].m_deviceSN;
        std::string sTopicTempName = sTopicTopName + "/depth_info";
        mapRosPublisher.insert(std::pair<PUBLISHER_TYPE, rclcpp::Publisher<CameraInfo>::SharedPtr>(DEPTH, m_node->create_publisher<CameraInfo>(sTopicTempName.c_str(), 1)));
        // ir
        sTopicTempName = sTopicTopName + "/ir_info";
        mapRosPublisher.insert(std::pair<PUBLISHER_TYPE, rclcpp::Publisher<CameraInfo>::SharedPtr>(IR, m_node->create_publisher<CameraInfo>(sTopicTempName.c_str(), 1)));

        // image pulisher
        // depth
        sTopicTempName = sTopicTopName + "/depth_raw";
        mapImagePublisher.insert(std::pair<PUBLISHER_TYPE, image_transport::Publisher>(DEPTH, m_imageTranspoort.advertise(sTopicTempName.c_str(), 1)));
        // ir
        sTopicTempName = sTopicTopName + "/ir_raw";
        mapImagePublisher.insert(std::pair<PUBLISHER_TYPE, image_transport::Publisher>(IR, m_imageTranspoort.advertise(sTopicTempName.c_str(), 1)));

        // rgb
        if ((m_pDeviceInfo[i].m_deviceType == Synexens::SYDEVICETYPE_CS30_DUAL || m_pDeviceInfo[i].m_deviceType == Synexens::SYDEVICETYPE_CS30_SINGLE || m_pDeviceInfo[i].m_deviceType == Synexens::SYDEVICETYPE_CS40PRO))
        {
            sTopicTempName = sTopicTopName + "/rgb_raw";
            mapImagePublisher.insert(std::pair<PUBLISHER_TYPE, image_transport::Publisher>(RGB, m_imageTranspoort.advertise(sTopicTempName.c_str(), 1)));
        }

        sTopicTempName = sTopicTopName + "/points2";
        mapPointsPublisher.insert(std::pair<PUBLISHER_TYPE, rclcpp::Publisher<PointCloud2>::SharedPtr>(POINTS, m_node->create_publisher<PointCloud2>(sTopicTempName.c_str(), 1)));
        m_mapPointsPublisher.insert(std::pair<int, std::map<PUBLISHER_TYPE, rclcpp::Publisher<PointCloud2>::SharedPtr>>(m_pDeviceInfo[i].m_nDeviceID, mapPointsPublisher));

        m_mapRosPublisher.insert(std::pair<int, std::map<PUBLISHER_TYPE, rclcpp::Publisher<CameraInfo>::SharedPtr>>(m_pDeviceInfo[i].m_nDeviceID, mapRosPublisher));
        m_mapImagePublisher.insert(std::pair<int, std::map<PUBLISHER_TYPE, image_transport::Publisher>>(m_pDeviceInfo[i].m_nDeviceID, mapImagePublisher));
        // ============= register topic ============= //
    }

    RCLCPP_INFO(m_node->get_logger(), "RegisterTopic Out");
}

void SYRosDevice::GetHWVersion()
{
    for (int i = 0; i < m_nCount; i++)
    {
        // get version
        int nStringLength = 0;
        Synexens::SYErrorCode errorCodeGetHWVersion = Synexens::GetDeviceHWVersion(m_pDeviceInfo[i].m_nDeviceID, nStringLength, nullptr);
        if (errorCodeGetHWVersion == Synexens::SYERRORCODE_SUCCESS)
        {
            if (nStringLength > 0)
            {
                char *pStringFWVersion = new char[nStringLength];
                errorCodeGetHWVersion = Synexens::GetDeviceHWVersion(m_pDeviceInfo[i].m_nDeviceID, nStringLength, pStringFWVersion);
                if (errorCodeGetHWVersion == Synexens::SYERRORCODE_SUCCESS)
                {
                    RCLCPP_INFO(m_node->get_logger(), "HWVersion%d:%s\n", i, pStringFWVersion);
                }
                else
                {
                    PrintErrorCode("GetDeviceHWVersion2", errorCodeGetHWVersion);
                }
                delete[] pStringFWVersion;
            }
        }
        else
        {
            PrintErrorCode("GetDeviceHWVersion", errorCodeGetHWVersion);
        }
    }
}

void SYRosDevice::ImportConfigurationStream()
{
    // get package path
    std::string sCameraConfig = ament_index_cpp::get_package_share_directory("synexens_ros2") + "/camera_config/";
    // get default config path
    std::string ConfigPath = std::string(sCameraConfig) + "default.yaml";

    Synexens::SYConfiguration configuration;
    if (Synexens::ParseConfiguration(ConfigPath.length(), ConfigPath.c_str(), configuration) != Synexens::SYERRORCODE_SUCCESS)
    {
        RCLCPP_ERROR(m_node->get_logger(), "get default config ERROR");
        return;
    }

    /* ======
    This is only an example. All modules are using the default configuration file.
    Custom configurations can be set for different modules based on SN, DeviceID, and device type.
    ====== */
    for (int i = 0; i < m_nCount; i++)
    {
        int nDeviceID = m_pDeviceInfo[i].m_nDeviceID;

        if (Synexens::ParseConfiguration(ConfigPath.length(), ConfigPath.c_str(), configuration) == Synexens::SYERRORCODE_SUCCESS)
        {
            Synexens::SYErrorCode errorcodeImport = Synexens::StartStreamingWithConfiguration(m_pDeviceInfo[i].m_nDeviceID, configuration);
            if (errorcodeImport == Synexens::SYERRORCODE_SUCCESS)
            {
                m_pOpen[i] = true;

                // 记录开流类型
                auto itStreamFind = m_mapStreamType.find(m_pDeviceInfo[i].m_nDeviceID);
                if (itStreamFind != m_mapStreamType.end())
                {
                    itStreamFind->second = configuration.m_streamType;
                }
                else
                {
                    m_mapStreamType.insert(std::pair<unsigned int, Synexens::SYStreamType>(m_pDeviceInfo[i].m_nDeviceID, configuration.m_streamType));
                }

                RCLCPP_INFO(m_node->get_logger(), "Import DeviceID:%d-SN:%s configuration success", m_pDeviceInfo[i].m_nDeviceID, m_mapRosDeviceInfo[nDeviceID].m_deviceSN.c_str());
            }
            else
            {
                RCLCPP_ERROR(m_node->get_logger(), "Import DeviceID:%d-SN:%s configuration Faild Code:%d", m_pDeviceInfo[i].m_nDeviceID, m_mapRosDeviceInfo[nDeviceID].m_deviceSN.c_str(), errorcodeImport);
            }
        }
        else
        {
            RCLCPP_ERROR(m_node->get_logger(), "ParseConfiguration error:%s", ConfigPath.c_str());
        }
    }
}

void SYRosDevice::ProcessFrameData(unsigned int nDeviceID, Synexens::SYFrameData *pFrameData)
{
    auto itStreamFind = m_mapStreamType.find(nDeviceID);
    if (itStreamFind == m_mapStreamType.end())
    {
        return;
    }

    // ros::Time capture_time;
    rclcpp::Time capture_time = m_node->now();
    if (itStreamFind->second == Synexens::SYSTREAMTYPE_RGBD)
    {

        std::map<Synexens::SYFrameType, int> mapIndex;
        std::map<Synexens::SYFrameType, int> mapPos;
        int nPos = 0;
        // Set mapping direction to true DepthtoRGB, false RGBtoDepth
        bool bTOFtoRGB = m_cameraParams.mapping_mode == 1 ? true : false;

        for (int nFrameIndex = 0; nFrameIndex < pFrameData->m_nFrameCount; nFrameIndex++)
        {
            mapIndex.insert(std::pair<Synexens::SYFrameType, int>(pFrameData->m_pFrameInfo[nFrameIndex].m_frameType, nFrameIndex));
            mapPos.insert(std::pair<Synexens::SYFrameType, int>(pFrameData->m_pFrameInfo[nFrameIndex].m_frameType, nPos));
            nPos += pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameHeight * pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameWidth * sizeof(short);
        }

        auto itDepthIndex = mapIndex.find(Synexens::SYFRAMETYPE_DEPTH);
        auto itRGBIndex = mapIndex.find(Synexens::SYFRAMETYPE_RGB);
        int nRGBDWidth = 0;
        int nRGBDHeight = 0;
        if (bTOFtoRGB)
        {
            nRGBDWidth = pFrameData->m_pFrameInfo[itRGBIndex->second].m_nFrameWidth;
            nRGBDHeight = pFrameData->m_pFrameInfo[itRGBIndex->second].m_nFrameHeight;
        }
        else
        {
            nRGBDWidth = pFrameData->m_pFrameInfo[itDepthIndex->second].m_nFrameWidth;
            nRGBDHeight = pFrameData->m_pFrameInfo[itDepthIndex->second].m_nFrameHeight;
        }

        unsigned short *pRGBDDepth = new unsigned short[nRGBDWidth * nRGBDHeight];
        memset(pRGBDDepth, 0, sizeof(unsigned short) * nRGBDWidth * nRGBDHeight);
        unsigned char *pRGBDRGB = new unsigned char[nRGBDWidth * nRGBDHeight * 3];
        memset(pRGBDRGB, 0, sizeof(unsigned char) * nRGBDWidth * nRGBDHeight * 3);

        if (itDepthIndex != mapIndex.end() && itRGBIndex != mapIndex.end())
        {
            if (Synexens::GetRGBD(nDeviceID, pFrameData->m_pFrameInfo[itDepthIndex->second].m_nFrameWidth, pFrameData->m_pFrameInfo[itDepthIndex->second].m_nFrameHeight, (unsigned short *)pFrameData->m_pData + mapPos[Synexens::SYFRAMETYPE_DEPTH],
                                  pFrameData->m_pFrameInfo[itRGBIndex->second].m_nFrameWidth, pFrameData->m_pFrameInfo[itRGBIndex->second].m_nFrameHeight, (unsigned char *)pFrameData->m_pData + mapPos[Synexens::SYFRAMETYPE_RGB],
                                  nRGBDWidth, nRGBDHeight, pRGBDDepth, pRGBDRGB, bTOFtoRGB) == Synexens::SYERRORCODE_SUCCESS)
            {
                int nCount = nRGBDWidth * nRGBDHeight;
                // device resolution
                SYResolution resolution;
                SYErrorCode errorCode = Synexens::GetFrameResolution(nDeviceID, SYFRAMETYPE_DEPTH, resolution);

                // camera intrinsics
                SYIntrinsics intrinsics;
                errorCode = Synexens::GetIntric(nDeviceID, resolution, false, intrinsics);
                if (errorCode != SYERRORCODE_SUCCESS)
                {
                    return;
                }

                unsigned char *pColor = new unsigned char[nCount * 3];
                cv::Mat gray16(nRGBDHeight, nRGBDWidth, CV_16UC1, pRGBDDepth);
                cv::Mat rgbimg = cv::Mat(nRGBDHeight, nRGBDWidth, CV_8UC3);
                cv::Mat bgrimg = cv::Mat(nRGBDHeight, nRGBDWidth, CV_8UC3);
                if (Synexens::GetDepthColor(nDeviceID, nCount, pRGBDDepth, pColor) == Synexens::SYERRORCODE_SUCCESS)
                {
                    memcpy(bgrimg.data, pColor, nCount * 3);
                    cv::cvtColor(bgrimg, rgbimg, cv::ColorConversionCodes::COLOR_RGB2BGR);
                }
                else
                {
                    cv::Mat tmp;
                    cv::Mat gray8 = cv::Mat(gray16.size(), CV_8U);
                    cv::normalize(gray16, tmp, 0, 255, cv::NORM_MINMAX);
                    cv::convertScaleAbs(tmp, gray8);
                    cv::cvtColor(gray8, rgbimg, cv::COLOR_GRAY2RGB);
                }
                delete[] pColor;

                // cv::Mat depth_frame_buffer_mat(nRGBDHeight, nRGBDWidth, CV_16UC1, pRGBDDepth);
                Image::SharedPtr depthImage = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_8SC3, rgbimg).toImageMsg();

                // point
                if (m_cameraParams.point_cloud_enabled)
                {
                    // get point
                    SYPointCloudData *pPCLData = new SYPointCloudData[nCount];
                    errorCode = Synexens::GetDepthPointCloud(nDeviceID, nRGBDWidth, nRGBDHeight, pRGBDDepth, pPCLData, false);
                    if (errorCode != SYERRORCODE_SUCCESS)
                    {
                        delete[] pPCLData;
                        return;
                    }
                    // point
                    PointCloud2::SharedPtr pointCloud(new msg::PointCloud2);
                    FillPointCloud(nRGBDWidth, nRGBDHeight, pPCLData, pointCloud);
                    pointCloud->header.frame_id = m_calibrationData.m_tfPrefix + m_calibrationData.m_depthCameraFrame;
                    pointCloud->header.stamp = capture_time;
                    m_mapPointsPublisher[nDeviceID][POINTS]->publish(*pointCloud.get());

                    delete[] pPCLData;
                }

                // frame_id stamp ros publish
                depthImage->header.stamp = capture_time;
                depthImage->header.frame_id = m_calibrationData.m_tfPrefix + m_calibrationData.m_depthCameraFrame;
                m_mapImagePublisher[nDeviceID][DEPTH].publish(depthImage);

                m_calibrationData.getDepthCameraInfo(m_depthCameraInfo, &intrinsics);
                m_mapRosPublisher[nDeviceID][DEPTH]->publish(m_depthCameraInfo);

                // rgb
                cv::Mat bgrImgRGB(nRGBDHeight, nRGBDWidth, CV_8UC3, pRGBDRGB);
                cv::Mat rgbImage;
                cvtColor(bgrImgRGB, rgbImage, cv::COLOR_BGR2RGB);

                Image::SharedPtr rgbImagePtr = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8, rgbImage).toImageMsg();
                m_mapImagePublisher[nDeviceID][RGB].publish(rgbImagePtr);
            }
        }

        delete[] pRGBDDepth;
        delete[] pRGBDRGB;
    }
    else
    {
        int nPos = 0;
        for (int nFrameIndex = 0; nFrameIndex < pFrameData->m_nFrameCount; nFrameIndex++)
        {
            int nCount = pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameWidth * pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameHeight;
            switch (pFrameData->m_pFrameInfo[nFrameIndex].m_frameType)
            {
            case Synexens::SYFRAMETYPE_DEPTH:
            {
                // unsigned char *pColor = new unsigned char[nCount * 3];

                // cv::Mat depth_frame_buffer_mat(pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameHeight, pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameWidth, CV_16UC1, (unsigned char *)pFrameData->m_pData + nPos);
                // cv::Mat rgbimg = cv::Mat(pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameHeight, pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameWidth, CV_8UC3);
                // cv::Mat bgrimg = cv::Mat(pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameHeight, pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameWidth, CV_8UC3);
                // if (Synexens::GetDepthColor(nDeviceID, nCount, (unsigned short *)pFrameData->m_pData + nPos, pColor) == Synexens::SYERRORCODE_SUCCESS)
                // {
                //     memcpy(rgbimg.data, pColor, nCount * 3);
                //     cv::cvtColor(rgbimg, bgrimg, cv::ColorConversionCodes::COLOR_RGB2BGR);
                //     Image::SharedPtr depthImage = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_16UC1, rgbimg).toImageMsg();
                // }
                // else
                // {
                //     Image::SharedPtr depthImage = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_16UC1, depth_frame_buffer_mat).toImageMsg();
                // }

                // delete[] pColor;

                unsigned char *pColor = new unsigned char[nCount * 3];
                cv::Mat gray16(pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameHeight, pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameWidth, CV_16UC1, (unsigned char *)pFrameData->m_pData + nPos);
                cv::Mat rgbimg = cv::Mat(pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameHeight, pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameWidth, CV_8UC3);
                cv::Mat bgrimg = cv::Mat(pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameHeight, pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameWidth, CV_8UC3);
                if (Synexens::GetDepthColor(nDeviceID, nCount, (unsigned short *)((unsigned char *)pFrameData->m_pData + nPos), pColor) == Synexens::SYERRORCODE_SUCCESS)
                {
                    memcpy(bgrimg.data, pColor, nCount * 3);
                    cv::cvtColor(bgrimg, rgbimg, cv::ColorConversionCodes::COLOR_RGB2BGR);
                }
                else
                {
                    cv::Mat tmp;
                    cv::Mat gray8 = cv::Mat(gray16.size(), CV_8U);
                    cv::normalize(gray16, tmp, 0, 255, cv::NORM_MINMAX);
                    cv::convertScaleAbs(tmp, gray8);
                    cv::cvtColor(gray8, rgbimg, cv::COLOR_GRAY2RGB);
                }
                delete[] pColor;

                Image::SharedPtr depthImage = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_8SC3, rgbimg).toImageMsg();

                // device resolution
                SYResolution resolution;
                SYErrorCode errorCode = SYERRORCODE_FAILED;
                errorCode = Synexens::GetFrameResolution(nDeviceID, SYFRAMETYPE_DEPTH, resolution);
                if (errorCode != SYERRORCODE_SUCCESS)
                {
                    continue;
                }

                // camera intrinsics
                SYIntrinsics intrinsics;
                errorCode = Synexens::GetIntric(nDeviceID, resolution, false, intrinsics);
                if (errorCode != SYERRORCODE_SUCCESS)
                {
                    continue;
                }

                // point
                if (m_cameraParams.point_cloud_enabled)
                {
                    // get point
                    SYPointCloudData *pPCLData = new SYPointCloudData[nCount];
                    errorCode = Synexens::GetDepthPointCloud(nDeviceID, pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameWidth, pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameHeight, (unsigned short *)((unsigned char *)pFrameData->m_pData + nPos), pPCLData, false);
                    if (errorCode != SYERRORCODE_SUCCESS)
                    {
                        delete[] pPCLData;
                        continue;
                    }
                    // point
                    PointCloud2::SharedPtr pointCloud(new msg::PointCloud2);
                    FillPointCloud(pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameWidth, pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameHeight, pPCLData, pointCloud);
                    pointCloud->header.frame_id = m_calibrationData.m_tfPrefix + m_calibrationData.m_depthCameraFrame;
                    pointCloud->header.stamp = capture_time;

                    m_mapPointsPublisher[nDeviceID][POINTS]->publish(*pointCloud.get());

                    delete[] pPCLData;
                }

                // frame_id stamp ros publish
                depthImage->header.stamp = capture_time;
                depthImage->header.frame_id = m_calibrationData.m_tfPrefix + m_calibrationData.m_depthCameraFrame;
                m_mapImagePublisher[nDeviceID][DEPTH].publish(depthImage);

                m_calibrationData.getDepthCameraInfo(m_depthCameraInfo, &intrinsics);
                m_mapRosPublisher[nDeviceID][DEPTH]->publish(m_depthCameraInfo);

                nPos += pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameHeight * pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameWidth * sizeof(short);

                break;
            }
            case Synexens::SYFRAMETYPE_IR:
            {

                cv::Mat ir_buffer_mat(pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameHeight, pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameWidth, CV_16UC1, (unsigned char *)pFrameData->m_pData + nPos);
                Image::SharedPtr irImage = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO16, ir_buffer_mat).toImageMsg();

                // device resolution
                SYResolution resolution;
                SYErrorCode errorCode = Synexens::GetFrameResolution(nDeviceID, SYFRAMETYPE_IR, resolution);

                // camera intrinsics
                SYIntrinsics intrinsics;
                errorCode = Synexens::GetIntric(nDeviceID, resolution, false, intrinsics);
                if (errorCode != SYERRORCODE_SUCCESS)
                    continue;

                irImage->header.stamp = capture_time;
                irImage->header.frame_id = m_calibrationData.m_tfPrefix + m_calibrationData.m_depthCameraFrame;
                m_mapImagePublisher[nDeviceID][IR].publish(irImage);

                m_calibrationData.getDepthCameraInfo(m_irCameraInfo, &intrinsics);
                m_mapRosPublisher[nDeviceID][IR]->publish(m_irCameraInfo);

                nPos += pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameHeight * pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameWidth * sizeof(short);
                break;
            }
            case Synexens::SYFRAMETYPE_RGB:
            {
                cv::Mat yuvImg(pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameHeight, pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameWidth, CV_8UC2, (unsigned char *)pFrameData->m_pData + nPos);
                cv::Mat rgbImg;
                cv::Mat bgrImg;
                rgbImg = cv::Mat(pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameHeight, pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameWidth, CV_8UC3);
                bgrImg = cv::Mat(pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameHeight, pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameWidth, CV_8UC3);
                cv::cvtColor(yuvImg, rgbImg, cv::ColorConversionCodes::COLOR_YUV2RGB_YUYV);
                cv::cvtColor(rgbImg, bgrImg, cv::ColorConversionCodes::COLOR_RGB2BGR);

                Image::SharedPtr rgbImage = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8, bgrImg).toImageMsg();
                m_mapImagePublisher[nDeviceID][RGB].publish(rgbImage);

                nPos += pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameHeight * pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameWidth * 3 / 2;
                break;
            }
            }
        }
    }
}

void SYRosDevice::FillPointCloud(int nWidth, int nHeight, SYPointCloudData *pPCLData, PointCloud2::SharedPtr &pointCloud)
{
    pointCloud->height = nHeight;
    pointCloud->width = nWidth;
    pointCloud->is_dense = false;
    pointCloud->is_bigendian = false;

    sensor_msgs::PointCloud2Modifier pcd_modifier(*pointCloud);
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

    sensor_msgs::PointCloud2Iterator<float> iter_x(*pointCloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*pointCloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*pointCloud, "z");

    int nPointCount = nHeight * nWidth;
    pcd_modifier.resize(nPointCount);

    for (int i = 0; i < nPointCount; i++, ++iter_x, ++iter_y, ++iter_z)
    {
        if (pPCLData[i].m_fltZ <= 0.0f)
        {
            *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
        }
        else
        {
            constexpr float kMillimeterToMeter = 1.0 / 1000.0f;
            *iter_x = kMillimeterToMeter * pPCLData[i].m_fltX;
            *iter_y = kMillimeterToMeter * pPCLData[i].m_fltY;
            *iter_z = kMillimeterToMeter * pPCLData[i].m_fltZ;
        }
    }
}