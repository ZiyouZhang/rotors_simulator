#include "apriltag_detection.hpp"

#include <ros/console.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

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

#include <iostream>

static const std::string OPENCV_WINDOW = "Image window";

PoseDetector::PoseDetector() : it(nh)
{
    nh.setParam("/start_motion", false);

    // Subscrive to input video feed and publish output video feed
    imageSub = it.subscribeCamera("/firefly/camera_nadir/image_raw", 10, &PoseDetector::imageCallBack, this);
    imagePub = it.advertise("/image_converter/output_video", 10);
    predictionPub = nh.advertise<nav_msgs::Odometry>("/predicted_object_state", 10);
    predictionPub2 = nh.advertise<nav_msgs::Odometry>("/predicted_object_state2", 10);
    predictionPub3 = nh.advertise<nav_msgs::Odometry>("/predicted_object_state3", 10);
    detectionPub = nh.advertise<nav_msgs::Odometry>("/detected_object_state", 10);
    updatePub = nh.advertise<nav_msgs::Odometry>("/updated_object_state", 10);

    // Initialise new cv window
    cv::namedWindow(OPENCV_WINDOW);

    // Initialise apriltag detection
    td = apriltag_detector_create();
    tf = tag36h11_create();
    apriltag_detector_add_family(td, tf);

    // double K[9], fx_value, fy_value, cx_value, cy_value;
    // nh.getParam("/firefly/camera_nadir/camera_info/K", K);
    // // nh.getParam("/firefly/camera_nadir/camera_info/K[2]", fy_value);
    // // nh.getParam("/firefly/camera_nadir/camera_info/K[4]", cx_value);
    // // nh.getParam("/firefly/camera_nadir/camera_info/K[5]", cy_value);
    // fx_value = K[0];
    // fy_value = K[2];
    // cx_value = K[4];
    // cy_value = K[5];

    // std::cout << fx_value << std::endl
    //           << fy_value << std::endl
    //           << cx_value << std::endl
    //           << cy_value << std::endl
    //           << std::endl;

    // Initialse camera info
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
}

PoseDetector::~PoseDetector()
{
    cv::destroyWindow(OPENCV_WINDOW);
    tag36h11_destroy(tf);
    apriltag_detector_destroy(td);
}

void PoseDetector::imageCallBack(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::CameraInfoConstPtr &camera_info)
{
    geometry_msgs::Twist currentTwist;
    nav_msgs::Odometry detected_state;
    nav_msgs::Odometry predicted_state;
    nav_msgs::Odometry predicted_state2;
    nav_msgs::Odometry predicted_state3;
    nav_msgs::Odometry updated_state;

    ros::Time currentTime = ros::Time::now();

    detected_state.header.frame_id = "/detected_tag_box";
    detected_state.header.stamp = currentTime;

    predicted_state2.header.frame_id = "/predicted_tag_box2";
    predicted_state2.header.stamp = currentTime + ros::Duration(2.0 / 30);

    predicted_state3.header.frame_id = "/predicted_tag_box3";
    predicted_state3.header.stamp = currentTime + ros::Duration(3.0 / 30);

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

        // Display the detacted tag using cvs

        // line(cv_ptr->image, cv::Point(det->p[0][0], det->p[0][1]),
        //      cv::Point(det->p[1][0], det->p[1][1]),
        //      cv::Scalar(0, 0xff, 0), 2);
        // line(cv_ptr->image, cv::Point(det->p[0][0], det->p[0][1]),
        //      cv::Point(det->p[3][0], det->p[3][1]),
        //      cv::Scalar(0, 0, 0xff), 2);
        // line(cv_ptr->image, cv::Point(det->p[1][0], det->p[1][1]),
        //      cv::Point(det->p[2][0], det->p[2][1]),
        //      cv::Scalar(0xff, 0, 0), 2);
        // line(cv_ptr->image, cv::Point(det->p[2][0], det->p[2][1]),
        //      cv::Point(det->p[3][0], det->p[3][1]),
        //      cv::Scalar(0xff, 0, 0), 2);

        // std::stringstream ss;
        // ss << det->id;
        // cv::String text = ss.str();
        // int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
        // double fontscale = 1.0;
        // int baseline;
        // cv::Size textsize = getTextSize(text, fontface, fontscale, 2,
        //                                 &baseline);
        // putText(cv_ptr->image, text, cv::Point(det->c[0] - textsize.width / 2, det->c[1] + textsize.height / 2),
        //         fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);

        // First create an apriltag_detection_info_t struct using known parameters.
        info.det = det;
        double err = estimate_tag_pose(&info, &pose);

        Eigen::Matrix4d T_CT;
        T_CT << pose.R->data[0], pose.R->data[1], pose.R->data[2], pose.t->data[0],
            pose.R->data[3], pose.R->data[4], pose.R->data[5], pose.t->data[1],
            pose.R->data[6], pose.R->data[7], pose.R->data[8], pose.t->data[2],
            0, 0, 0, 1;

        // obtain the T_WC
        tf::StampedTransform transform_WC;
        listener.lookupTransform("/world", "/firefly/camera_nadir_optical_link", ros::Time(0), transform_WC);
        Eigen::Matrix4d T_WC;
        T_WC << transform_WC.getBasis()[0][0], transform_WC.getBasis()[0][1], transform_WC.getBasis()[0][2], transform_WC.getOrigin()[0],
            transform_WC.getBasis()[1][0], transform_WC.getBasis()[1][2], transform_WC.getBasis()[1][2], transform_WC.getOrigin()[1],
            transform_WC.getBasis()[2][0], transform_WC.getBasis()[2][1], transform_WC.getBasis()[2][2], transform_WC.getOrigin()[2],
            0, 0, 0, 1;

        // std::cout << transform.getBasis()[0][0] << " " << transform.getBasis()[0][1] << " " << transform.getBasis()[0][2] << std::endl
        //           << transform.getBasis()[1][0] << " " << transform.getBasis()[1][1] << " " << transform.getBasis()[1][2] << std::endl
        //           << transform.getBasis()[2][0] << " " << transform.getBasis()[2][1] << " " << transform.getBasis()[2][2] << std::endl
        // std::cout << transform.getOrigin()[0] << " " << transform.getOrigin()[1] << " " << transform.getOrigin()[2] << std::endl;

        T_WO = T_WC * T_CT * T_TO;

        // construct the detected tf and broadcast
        transform_WO.setOrigin(tf::Vector3(T_WO(0, 3), T_WO(1, 3), T_WO(2, 3)));
        transform_WO.setRotation(quaternion_WO);
        br.sendTransform(tf::StampedTransform(transform_WO, ros::Time::now(), "/world", "detected_tag_box"));

        detected_state.pose.pose.position.x = T_WO(0, 3);
        detected_state.pose.pose.position.y = T_WO(1, 3);
        detected_state.pose.pose.position.z = T_WO(2, 3);

        tf::Matrix3x3 rotationMatrix(T_WO(0, 0), T_WO(0, 1), T_WO(0, 2),
                                     T_WO(1, 0), T_WO(1, 1), T_WO(1, 2),
                                     T_WO(2, 0), T_WO(2, 1), T_WO(2, 2));
        rotationMatrix.getRotation(quaternion_WO);
        quaternion_WO.normalize();
        tf::quaternionTFToMsg(quaternion_WO, detected_state.pose.pose.orientation);

        listener.lookupTwist("/tag_box", "/world", ros::Time(0), ros::Duration(0.1), currentTwist);
        detected_state.twist.twist = currentTwist;
        detectionPub.publish(detected_state);

        if (!visualEKF.isInitialsed())
        {
            ObjectState initialState;
            initialState.r_W = Eigen::Vector3d(T_WO(0, 3), T_WO(1, 3), T_WO(2, 3));
            Eigen::Quaterniond eigen_quaterion(quaternion_WO.getW(),
                                               quaternion_WO.getAxis().getX(),
                                               quaternion_WO.getAxis().getY(),
                                               quaternion_WO.getAxis().getZ());
            initialState.q_WO = eigen_quaterion;
            initialState.v_O = Eigen::Vector3d(currentTwist.linear.x,
                                               currentTwist.linear.y,
                                               currentTwist.linear.z);
            initialState.omega_O = Eigen::Vector3d(currentTwist.angular.x,
                                                   currentTwist.angular.y,
                                                   currentTwist.angular.z);
            initialState.timestamp = currentTime.toSec();
            initialState.inertia << 0.5 / 12, 0, 0,
                0, 0.5 / 12, 0,
                0, 0, 0.5 / 12;

            visualEKF.initialise(initialState);

            // publish initial state
            updated_state.header.frame_id = "/updated_tag_box";
            updated_state.header.stamp = currentTime;
            updated_state.pose.pose.position.x = visualEKF.x_.r_W.x();
            updated_state.pose.pose.position.y = visualEKF.x_.r_W.y();
            updated_state.pose.pose.position.z = visualEKF.x_.r_W.z();
            updated_state.pose.pose.orientation.w = visualEKF.x_.q_WO.w();
            updated_state.pose.pose.orientation.x = visualEKF.x_.q_WO.x();
            updated_state.pose.pose.orientation.y = visualEKF.x_.q_WO.y();
            updated_state.pose.pose.orientation.z = visualEKF.x_.q_WO.z();
            updated_state.twist.twist.linear.x = visualEKF.x_.v_O.x();
            updated_state.twist.twist.linear.y = visualEKF.x_.v_O.y();
            updated_state.twist.twist.linear.z = visualEKF.x_.v_O.z();
            updated_state.twist.twist.angular.x = visualEKF.x_.omega_O.x();
            updated_state.twist.twist.angular.y = visualEKF.x_.omega_O.y();
            updated_state.twist.twist.angular.z = visualEKF.x_.omega_O.z();
            updatePub.publish(updated_state);

            // notify other process to start motion
            nh.setParam("/start_motion", true);

            // debug initial state
            // std::cout << std::endl
            //           << "r_W: " << std::endl
            //           << visualEKF.x_.r_W << std::endl
            //           << "q_WO: " << std::endl
            //           << visualEKF.x_.q_WO.w() << std::endl
            //           << visualEKF.x_.q_WO.vec() << std::endl
            //           << "v_O: " << std::endl
            //           << visualEKF.x_.v_O << std::endl
            //           << "omega_O: " << std::endl
            //           << visualEKF.x_.omega_O << std::endl
            //           << std::endl;
        }
        else
        {
            double dt = currentTime.toSec() - lastMeasurementTime;
            // std::cout << dt << std::endl;
            visualEKF.x_.timestamp = currentTime.toSec();
            visualEKF.x_predicted_.timestamp = currentTime.toSec();

            visualEKF.predict(1.0 / 30);
            predicted_state.header.frame_id = "/predicted_tag_box";
            predicted_state.header.stamp = currentTime;
            predicted_state.pose.pose.position.x = visualEKF.x_predicted_.r_W.x();
            predicted_state.pose.pose.position.y = visualEKF.x_predicted_.r_W.y();
            predicted_state.pose.pose.position.z = visualEKF.x_predicted_.r_W.z();
            predicted_state.pose.pose.orientation.w = visualEKF.x_predicted_.q_WO.w();
            predicted_state.pose.pose.orientation.x = visualEKF.x_predicted_.q_WO.x();
            predicted_state.pose.pose.orientation.y = visualEKF.x_predicted_.q_WO.y();
            predicted_state.pose.pose.orientation.z = visualEKF.x_predicted_.q_WO.z();
            predicted_state.twist.twist.linear.x = visualEKF.x_predicted_.v_O.x();
            predicted_state.twist.twist.linear.y = visualEKF.x_predicted_.v_O.y();
            predicted_state.twist.twist.linear.z = visualEKF.x_predicted_.v_O.z();
            predicted_state.twist.twist.angular.x = visualEKF.x_predicted_.omega_O.x();
            predicted_state.twist.twist.angular.y = visualEKF.x_predicted_.omega_O.y();
            predicted_state.twist.twist.angular.z = visualEKF.x_predicted_.omega_O.z();
            predictionPub.publish(predicted_state);

            // debug predicted state
            // std::cout << std::endl
            //           << "r_W: " << std::endl
            //           << visualEKF.x_predicted_.r_W << std::endl << detected_state.pose.pose.position
            //   << "q_WO: " << std::endl
            //   << visualEKF.x_predicted_.q_WO.w() << std::endl
            //   << visualEKF.x_predicted_.q_WO.vec() << std::endl
            //   << "v_O: " << std::endl
            //   << visualEKF.x_predicted_.v_O << std::endl
            //   << "omega_O: " << std::endl
            //   << visualEKF.x_predicted_.omega_O << std::endl
            //   << std::endl;

            ApriltagMeasurement measurement;
            measurement.r_W = Eigen::Vector3d(T_WO(0, 3), T_WO(1, 3), T_WO(2, 3));

            Eigen::Quaterniond eigen_quaterion(quaternion_WO.getW(),
                                               quaternion_WO.getAxis().getX(),
                                               quaternion_WO.getAxis().getY(),
                                               quaternion_WO.getAxis().getZ());
            measurement.q_WO = eigen_quaterion;
            visualEKF.update(measurement);

            updated_state.header.frame_id = "/updated_tag_box";
            updated_state.header.stamp = currentTime;
            updated_state.pose.pose.position.x = visualEKF.x_.r_W.x();
            updated_state.pose.pose.position.y = visualEKF.x_.r_W.y();
            updated_state.pose.pose.position.z = visualEKF.x_.r_W.z();
            updated_state.pose.pose.orientation.w = visualEKF.x_.q_WO.w();
            updated_state.pose.pose.orientation.x = visualEKF.x_.q_WO.x();
            updated_state.pose.pose.orientation.y = visualEKF.x_.q_WO.y();
            updated_state.pose.pose.orientation.z = visualEKF.x_.q_WO.z();
            updated_state.twist.twist.linear.x = visualEKF.x_.v_O.x();
            updated_state.twist.twist.linear.y = visualEKF.x_.v_O.y();
            updated_state.twist.twist.linear.z = visualEKF.x_.v_O.z();
            updated_state.twist.twist.angular.x = visualEKF.x_.omega_O.x();
            updated_state.twist.twist.angular.y = visualEKF.x_.omega_O.y();
            updated_state.twist.twist.angular.z = visualEKF.x_.omega_O.z();
            updatePub.publish(updated_state);
        }

        // visualEKF.x_.r_W = Eigen::Vector3d(T_WO(0, 3), T_WO(1, 3), T_WO(2, 3));
        // Eigen::Quaterniond eigen_quaterion(quaternion_WO.getW(),
        //                                    quaternion_WO.getAxis().getX(),
        //                                    quaternion_WO.getAxis().getY(),
        //                                    quaternion_WO.getAxis().getZ());
        // visualEKF.x_.q_WO = eigen_quaterion;
        // visualEKF.x_.v_O = Eigen::Vector3d(currentTwist.linear.x,
        //                                    currentTwist.linear.y,
        //                                    currentTwist.linear.z);
        // visualEKF.x_.omega_O = Eigen::Vector3d(currentTwist.angular.x,
        //                                        currentTwist.angular.y,
        //                                        currentTwist.angular.z);

        // visualEKF.predict(2.0 / 30);
        // visualEKF.x_predicted_.timestamp += 2.0 / 60;
        // predicted_state2.pose.pose.position.x = visualEKF.x_predicted_.r_W.x();
        // predicted_state2.pose.pose.position.y = visualEKF.x_predicted_.r_W.y();
        // predicted_state2.pose.pose.position.z = visualEKF.x_predicted_.r_W.z();
        // predicted_state2.pose.pose.orientation.w = visualEKF.x_predicted_.q_WO.w();
        // predicted_state2.pose.pose.orientation.x = visualEKF.x_predicted_.q_WO.x();
        // predicted_state2.pose.pose.orientation.y = visualEKF.x_predicted_.q_WO.y();
        // predicted_state2.pose.pose.orientation.z = visualEKF.x_predicted_.q_WO.z();
        // predicted_state2.twist.twist.linear.x = visualEKF.x_predicted_.v_O.x();
        // predicted_state2.twist.twist.linear.y = visualEKF.x_predicted_.v_O.y();
        // predicted_state2.twist.twist.linear.z = visualEKF.x_predicted_.v_O.z();
        // predicted_state2.twist.twist.angular.x = visualEKF.x_predicted_.omega_O.x();
        // predicted_state2.twist.twist.angular.y = visualEKF.x_predicted_.omega_O.y();
        // predicted_state2.twist.twist.angular.z = visualEKF.x_predicted_.omega_O.z();
        // predictionPub2.publish(predicted_state2);

        // visualEKF.predict(3.0 / 30);
        // visualEKF.x_predicted_.timestamp += 3.0 / 60;
        // predicted_state3.pose.pose.position.x = visualEKF.x_predicted_.r_W.x();
        // predicted_state3.pose.pose.position.y = visualEKF.x_predicted_.r_W.y();
        // predicted_state3.pose.pose.position.z = visualEKF.x_predicted_.r_W.z();
        // predicted_state3.pose.pose.orientation.w = visualEKF.x_predicted_.q_WO.w();
        // predicted_state3.pose.pose.orientation.x = visualEKF.x_predicted_.q_WO.x();
        // predicted_state3.pose.pose.orientation.y = visualEKF.x_predicted_.q_WO.y();
        // predicted_state3.pose.pose.orientation.z = visualEKF.x_predicted_.q_WO.z();
        // predicted_state3.twist.twist.linear.x = visualEKF.x_predicted_.v_O.x();
        // predicted_state3.twist.twist.linear.y = visualEKF.x_predicted_.v_O.y();
        // predicted_state3.twist.twist.linear.z = visualEKF.x_predicted_.v_O.z();
        // predicted_state3.twist.twist.angular.x = visualEKF.x_predicted_.omega_O.x();
        // predicted_state3.twist.twist.angular.y = visualEKF.x_predicted_.omega_O.y();
        // predicted_state3.twist.twist.angular.z = visualEKF.x_predicted_.omega_O.z();
        // predictionPub3.publish(predicted_state3);

        // debug detected pose
        // std::cout << detected_state.header.stamp.sec + detected_state.header.stamp.nsec * 10e-9 << "," << detected_state.pose.pose.position.z << "," << T_WO(2, 3) << std::endl;
    }

    apriltag_detections_destroy(detections);

    // Update GUI Window
    // cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    // cv::waitKey(3);

    imagePub.publish(cv_ptr->toImageMsg());

    // update the measurement time
    lastMeasurementTime = currentTime.toSec();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_converter");
    PoseDetector ic;
    ros::spin();
    return 0;
}
