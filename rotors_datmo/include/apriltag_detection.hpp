#ifndef APRILTAG_DETECTION_H
#define APRILTAG_DETECTION_H

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
// #include "apriltag_ros/common_functions.h"

class PoseDetector
{
public:
    PoseDetector();
    ~PoseDetector();
    void imageCallBack(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::CameraInfoConstPtr &camera_info);

private:
    ros::NodeHandle nh;
    ros::Publisher posePub;
    ros::Publisher twistPub;
    image_transport::ImageTransport it;
    image_transport::CameraSubscriber imageSub;
    image_transport::Publisher imagePub;

    tf::TransformBroadcaster br;
    tf::TransformListener listener;
    tf::Transform transform;
    tf::Quaternion quaternion;

    apriltag_detector_t *td;
    apriltag_family_t *tf;
    apriltag_pose_t pose;
    apriltag_detection_info_t info;

    Eigen::Matrix4d T_WO;

    Eigen::Matrix4d T_TO;

    // apriltag_ros::TagDetector tagDetector;
};

#endif // APRILTAG_DETECTION_H