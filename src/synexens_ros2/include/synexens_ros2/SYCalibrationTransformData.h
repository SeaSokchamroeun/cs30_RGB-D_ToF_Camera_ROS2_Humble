#ifndef SYCalibrationTransformData_H
#define SYCalibrationTransformData_H

#include <iostream>

#include "SYRosDeviceParmas.h"

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <angles/angles.h>

using namespace Synexens;
class SYCalibrationTransformData
{
private:
    /* data */
public:
    void initialize(const SYCameraParams &params, rclcpp::Node::SharedPtr nd);

    // set depth intrinsics params
    void setDepthCameraCalib(const SYIntrinsics &intrinsics);
    // set rgb intrinsics params
    void setColorCameraCalib(const SYIntrinsics &intrinsics);
    // get depth width
    int getDepthWidth();
    // get depth height
    int getDepthHeight();
    // get rgb width
    int getColorWidth();
    // get rgb height
    int getColorHeight();
    // get depth camera info
    void getDepthCameraInfo(sensor_msgs::msg::CameraInfo &camera_info, SYIntrinsics *intrinsics = nullptr);
    // get rgb camera info
    void getRgbCameraInfo(sensor_msgs::msg::CameraInfo &camera_info, SYIntrinsics *intrinsics = nullptr);

    // set camera info
    static void setCameraInfo(const SYIntrinsics &parameters, sensor_msgs::msg::CameraInfo &camera_info);

    SYIntrinsics m_rgbCameraIntrinsics;
    SYIntrinsics m_depthCameraIntrinsics;

    std::string m_tfPrefix = "";
    std::string m_cameraBaseFrame = "camera_base";
    std::string m_rgbCameraFrame = "rgb_camera_link";
    std::string m_depthCameraFrame = "depth_camera_link";

private:
    void printCameraCalibration(SYIntrinsics &calibration);
    void publishDepthToBaseTf(rclcpp::Node::SharedPtr nd);

    tf2::Quaternion getDepthToBaseRotationCorrection();
    tf2::Vector3 getDepthToBaseTranslationCorrection();
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};

#endif