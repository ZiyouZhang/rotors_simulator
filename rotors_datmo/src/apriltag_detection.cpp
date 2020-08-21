#include "apriltag_detection.hpp"

#include <ros/console.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <ctype.h>
#include <unistd.h>
#include <math.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <tf/transform_datatypes.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

static const std::string OPENCV_WINDOW = "Image window";

PoseDetector::PoseDetector() : it(nh)
{
    // Subscrive to input video feed and publish output video feed
    imageSub = it.subscribeCamera("/firefly/camera_nadir/image_raw", 1, &PoseDetector::imageCallBack, this);
    imagePub = it.advertise("/image_converter/output_video", 1);
    posePub = nh.advertise<geometry_msgs::PoseStamped>("/tag_box_pose", 1);
    twistPub = nh.advertise<geometry_msgs::Twist>("/tag_box_twist", 1);

    // Initialise new cv window
    cv::namedWindow(OPENCV_WINDOW);

    // Initialise apriltag detection
    td = apriltag_detector_create();
    tf = tag36h11_create();
    apriltag_detector_add_family(td, tf);

    // Initialse camera info

    // std_msgs::Float64 fx_value, fy_value, cx_value, cy_value;
    // nh.getParam("/firefly/camera_nadir/camera_info/K[0]", fx_value);
    // nh.getParam("/firefly/camera_nadir/camera_info/K[2]", fy_value);
    // nh.getParam("/firefly/camera_nadir/camera_info/K[4]", cx_value);
    // nh.getParam("/firefly/camera_nadir/camera_info/K[5]", cy_value);

    // info.tagsize = 0.4;
    // info.fx = static_cast<double>(fx_value);
    // info.fy = static_cast<double>(fy_value);
    // info.cx = static_cast<double>(cx_value);
    // info.cy = static_cast<double>(cy_value);

    info.tagsize = 0.4;
    info.fx = 554.3827128226441;
    info.fy = 554.3827128226441;
    info.cx = 320.5;
    info.cy = 240.5;

    // Initialise transformation matrices
    T_TO << 0, -1, 0, 0,
        0, 0, -1, 0,
        1, 0, 0, 0.25,
        0, 0, 0, 1;

    // Initialise measurement
    visualEKF.lastApriltagMeasurement.timestamp = 0;
    visualEKF.lastApriltagMeasurement.r_W(0) = 2.0;
    visualEKF.lastApriltagMeasurement.r_W(1) = -0.5;
    visualEKF.lastApriltagMeasurement.r_W(2) = 2.5;
    visualEKF.lastApriltagMeasurement.q_WO = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);

    visualEKF.x_.inertia << 0.5 / 3, 0, 0,
                            0, 0.5 / 3, 0,
                            0, 0, 0.5 / 3;

    recordFile.open("~/Desktop/log.txt");
}

PoseDetector::~PoseDetector()
{
    cv::destroyWindow(OPENCV_WINDOW);
    tag36h11_destroy(tf);
    apriltag_detector_destroy(td);

    recordFile.close();
}

void PoseDetector::imageCallBack(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::CameraInfoConstPtr &camera_info)
{
    geometry_msgs::PoseStamped currentPose;
    currentPose.header.frame_id = "/detected_tag_box";
    currentPose.header.stamp = ros::Time::now();

    visualEKF.currentApriltagMeasurement.timestamp = ros::Time::now().toSec();

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Convert cv::Mat to image_u8_t for apriltag package
    image_u8_t img_header = {.width = cv_ptr->image.cols,
                             .height = cv_ptr->image.rows,
                             .stride = cv_ptr->image.cols,
                             .buf = cv_ptr->image.data};

    zarray_t *detections = apriltag_detector_detect(td, &img_header);

    for (int i = 0; i < zarray_size(detections); i++)
    {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);

        line(cv_ptr->image, cv::Point(det->p[0][0], det->p[0][1]),
             cv::Point(det->p[1][0], det->p[1][1]),
             cv::Scalar(0, 0xff, 0), 2);
        line(cv_ptr->image, cv::Point(det->p[0][0], det->p[0][1]),
             cv::Point(det->p[3][0], det->p[3][1]),
             cv::Scalar(0, 0, 0xff), 2);
        line(cv_ptr->image, cv::Point(det->p[1][0], det->p[1][1]),
             cv::Point(det->p[2][0], det->p[2][1]),
             cv::Scalar(0xff, 0, 0), 2);
        line(cv_ptr->image, cv::Point(det->p[2][0], det->p[2][1]),
             cv::Point(det->p[3][0], det->p[3][1]),
             cv::Scalar(0xff, 0, 0), 2);

        std::stringstream ss;
        ss << det->id;
        cv::String text = ss.str();
        int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
        double fontscale = 1.0;
        int baseline;
        cv::Size textsize = getTextSize(text, fontface, fontscale, 2,
                                        &baseline);
        putText(cv_ptr->image, text, cv::Point(det->c[0] - textsize.width / 2, det->c[1] + textsize.height / 2),
                fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);

        // First create an apriltag_detection_info_t struct using known parameters.
        info.det = det;
        double err = estimate_tag_pose(&info, &pose);

        Eigen::Matrix4d T_CT;
        T_CT << pose.R->data[0], pose.R->data[1], pose.R->data[2], pose.t->data[0],
            pose.R->data[3], pose.R->data[4], pose.R->data[5], pose.t->data[1],
            pose.R->data[6], pose.R->data[7], pose.R->data[8], pose.t->data[2],
            0, 0, 0, 1;

        T_WO = T_CT * T_TO;

        currentPose.pose.position.x = T_WO(0, 3);
        currentPose.pose.position.y = T_WO(1, 3);
        currentPose.pose.position.z = T_WO(2, 3);

        tf::Matrix3x3 rotationMatrix(T_WO(0, 0), T_WO(0, 1), T_WO(0, 2),
                                     T_WO(1, 0), T_WO(1, 1), T_WO(1, 2),
                                     T_WO(2, 0), T_WO(2, 1), T_WO(2, 2));

        rotationMatrix.getRotation(quaternion);
        tf::quaternionTFToMsg(quaternion, currentPose.pose.orientation);

        // // transformStuff
        // tf::StampedTransform transform;

        visualEKF.currentApriltagMeasurement.r_W = Eigen::Vector3d(T_WO(0, 3), T_WO(1, 3), T_WO(2, 3));
        Eigen::Quaterniond eigen_quaterion(quaternion.getW(),
                                           quaternion.getAxis().getX(),
                                           quaternion.getAxis().getY(),
                                           quaternion.getAxis().getZ());
        visualEKF.currentApriltagMeasurement.q_WO = eigen_quaterion;
        visualEKF.predict();

        std::cout << visualEKF.x_.r_W(0)
                  << visualEKF.x_.r_W(1)
                  << visualEKF.x_.r_W(2)
                  << visualEKF.x_.q_WO.w()
                  << visualEKF.x_.q_WO.vec()
                  << visualEKF.x_.v_O(0)
                  << visualEKF.x_.v_O(1)
                  << visualEKF.x_.v_O(2)
                  << visualEKF.x_.omega_O(0)
                  << visualEKF.x_.omega_O(1)
                  << visualEKF.x_.omega_O(2)
                  << "\n";

        // ROS_INFO( visualEKF.x_.r_W(0),
        //           visualEKF.x_.r_W(1),
        //           visualEKF.x_.r_W(2),
        //           visualEKF.x_.q_WO.w(),
        //           visualEKF.x_.q_WO.vec(),
        //           visualEKF.x_.v_O(0),
        //           visualEKF.x_.v_O(1),
        //           visualEKF.x_.v_O(2),
        //           visualEKF.x_.omega_O(0),
        //           visualEKF.x_.omega_O(1),
        //           visualEKF.x_.omega_O(2),
        //           "\n");

        visualEKF.lastApriltagMeasurement = visualEKF.lastApriltagMeasurement;

        transform.setOrigin(tf::Vector3(T_WO(0, 3), T_WO(1, 3), T_WO(2, 3)));

        transform.setRotation(quaternion);

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "firefly/camera_nadir_optical_link", "detected_tag_box"));

        geometry_msgs::Twist currentTwist;
        listener.lookupTwist("/tag_box", "/world", "/tag_box", tf::Point(), "/world", ros::Time(0), ros::Duration(0.1), currentTwist);

        posePub.publish(currentPose);
        twistPub.publish(currentTwist);
    }

    apriltag_detections_destroy(detections);

    // apriltag_ros::AprilTagDetectionArray detections = tagDetector.detectTags(cv_ptr, camera_info);
    // for (auto i : detections.detections) {
    //     std::cout << i << std::endl;
    // }

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    imagePub.publish(cv_ptr->toImageMsg());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_converter");
    PoseDetector ic;
    ros::spin();
    return 0;
}
