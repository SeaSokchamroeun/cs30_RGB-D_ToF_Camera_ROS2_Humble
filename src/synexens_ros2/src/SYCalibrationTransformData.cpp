#include "synexens_ros2/SYCalibrationTransformData.h"

void SYCalibrationTransformData::initialize(const SYCameraParams &params, rclcpp::Node::SharedPtr nd)
{
    m_tfPrefix = params.tf_prefix;
    publishDepthToBaseTf(nd);
}

void SYCalibrationTransformData::setDepthCameraCalib(const SYIntrinsics &intrinsics)
{
    m_depthCameraIntrinsics = intrinsics;
}

void SYCalibrationTransformData::setColorCameraCalib(const SYIntrinsics &intrinsics)
{
    m_rgbCameraIntrinsics = intrinsics;
}


int SYCalibrationTransformData::getDepthWidth()
{
    return m_depthCameraIntrinsics.m_nWidth;
}

int SYCalibrationTransformData::getDepthHeight()
{
    return m_depthCameraIntrinsics.m_nHeight;
}

int SYCalibrationTransformData::getColorWidth()
{
    return m_rgbCameraIntrinsics.m_nWidth;
}

int SYCalibrationTransformData::getColorHeight()
{
    return m_rgbCameraIntrinsics.m_nHeight;
}

void SYCalibrationTransformData::setCameraInfo(const SYIntrinsics &parameters, sensor_msgs::msg::CameraInfo &camera_info)
{
    camera_info.d = {parameters.m_fltCoeffs[0], parameters.m_fltCoeffs[1], parameters.m_fltCoeffs[2],
                     parameters.m_fltCoeffs[3], parameters.m_fltCoeffs[4]};

    camera_info.k = {parameters.m_fltFocalDistanceX, 0.0f, parameters.m_fltCenterPointX,
                     0.0f, parameters.m_fltFocalDistanceY, parameters.m_fltCenterPointY,
                     0.0f, 0.0, 1.0f};

    camera_info.p = {parameters.m_fltFocalDistanceX, 0.0f, parameters.m_fltCenterPointX, 0.0f,
                     0.0f, parameters.m_fltFocalDistanceY, parameters.m_fltCenterPointY, 0.0f,
                     0.0f, 0.0, 1.0f, 0.0f};

    camera_info.r = {1.0f, 0.0f, 0.0f,
                     0.0f, 1.0f, 0.0f,
                     0.0f, 0.0f, 1.0f};
}



void SYCalibrationTransformData::publishDepthToBaseTf(rclcpp::Node::SharedPtr nd)
{
    // This is a purely cosmetic transform to make the base model of the URDF look good.
    geometry_msgs::msg::TransformStamped static_transform;
    static_transform.header.stamp = rclcpp::Clock().now();
    static_transform.header.frame_id = m_tfPrefix + m_cameraBaseFrame;
    static_transform.child_frame_id = m_tfPrefix + m_depthCameraFrame;

    tf2::Vector3 depth_translation = getDepthToBaseTranslationCorrection();
    static_transform.transform.translation.x = depth_translation.x();
    static_transform.transform.translation.y = depth_translation.y();
    static_transform.transform.translation.z = depth_translation.z();

    tf2::Quaternion depth_rotation = getDepthToBaseRotationCorrection();
    static_transform.transform.rotation.x = depth_rotation.x();
    static_transform.transform.rotation.y = depth_rotation.y();
    static_transform.transform.rotation.z = depth_rotation.z();
    static_transform.transform.rotation.w = depth_rotation.w();

    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(nd);
    static_broadcaster_->sendTransform(static_transform);
}

// The [0,0,0] center of the URDF, the TF frame known as "camera_base", is offset slightly from the
// [0,0,0] origin of the depth camera frame, known as "depth_camera_link" or "depth_camera_frame"
//
// Publish a TF link so the URDF model and the depth camera line up correctly
#define DEPTH_CAMERA_OFFSET_MM_X 0.0f
#define DEPTH_CAMERA_OFFSET_MM_Y 0.0f
#define DEPTH_CAMERA_OFFSET_MM_Z 1.8f  // The depth camera is shifted 1.8mm up in the depth window


void SYCalibrationTransformData::getDepthCameraInfo(sensor_msgs::msg::CameraInfo &camera_info, SYIntrinsics *intrinsics)
{
    camera_info.header.frame_id = m_tfPrefix + m_depthCameraFrame;
    camera_info.width = intrinsics ? intrinsics->m_nWidth : getColorWidth();
    camera_info.height = intrinsics ? intrinsics->m_nHeight : getColorHeight();
    camera_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    setCameraInfo(intrinsics ? *intrinsics : m_depthCameraIntrinsics, camera_info);
}

void SYCalibrationTransformData::getRgbCameraInfo(sensor_msgs::msg::CameraInfo &camera_info, SYIntrinsics *intrinsics)
{
    camera_info.header.frame_id = m_tfPrefix + m_rgbCameraFrame;
    camera_info.width = intrinsics ? intrinsics->m_nWidth : getColorWidth();
    camera_info.height = intrinsics ? intrinsics->m_nHeight : getColorHeight();
    camera_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    setCameraInfo(intrinsics ? *intrinsics : m_rgbCameraIntrinsics, camera_info);
}

tf2::Vector3 SYCalibrationTransformData::getDepthToBaseTranslationCorrection()
{
    // These are purely cosmetic tranformations for the URDF drawing!!
    return tf2::Vector3(DEPTH_CAMERA_OFFSET_MM_X / 1000.0f, DEPTH_CAMERA_OFFSET_MM_Y / 1000.0f,
                        DEPTH_CAMERA_OFFSET_MM_Z / 1000.0f);
}

tf2::Quaternion SYCalibrationTransformData::getDepthToBaseRotationCorrection()
{
    // These are purely cosmetic tranformations for the URDF drawing!!
    tf2::Quaternion ros_camera_rotation; // ROS camera co-ordinate system requires rotating the entire camera relative to
                                         // camera_base
    tf2::Quaternion depth_rotation;      // K4A has one physical camera that is about 6 degrees downward facing.

    depth_rotation.setEuler(0, angles::from_degrees(-6.0), 0);
    ros_camera_rotation.setEuler(M_PI / -2.0f, M_PI, (M_PI / 2.0f));

    return ros_camera_rotation * depth_rotation;
}
