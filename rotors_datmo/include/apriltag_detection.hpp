#ifndef APRILTAG_DETECTION_H_
#define APRILTAG_DETECTION_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>

#include <Eigen/Dense>

#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include "apriltag/apriltag_pose.h"
#include "apriltag/apriltag_math.h"

#include "visual_ekf.hpp"

class PoseDetector
{
public:
    PoseDetector();
    ~PoseDetector();
    void imageCallBack(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::CameraInfoConstPtr &camera_info);

private:
    ros::NodeHandle nh;
    ros::Publisher predictionPub;
    ros::Publisher detectionPub;

    image_transport::ImageTransport it;
    image_transport::CameraSubscriber imageSub;
    image_transport::Publisher imagePub;

    tf::TransformBroadcaster br;
    tf::TransformListener listener;
    tf::Transform transform_WO;
    tf::Quaternion quaternion_WO;

    apriltag_detector_t *td;
    apriltag_family_t *tf;
    apriltag_pose_t pose;
    apriltag_detection_info_t info;

    Eigen::Matrix4d T_WO;
    Eigen::Matrix4d T_TO;

    VisualEKF visualEKF;
    uint64_t lastImageTime;
    uint64_t currentImageTime;
};

#endif // APRILTAG_DETECTION_H_