#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include "apriltag/apriltag_pose.h"
#include "apriltag/apriltag_math.h"

// #include "apriltag_ros/common_functions.h"

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <ctype.h>
#include <unistd.h>
#include <math.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class PoseDetector
{
public:
    PoseDetector()
        // : it(nh), tagDetector(nh)
        : it(nh)
    {
        // Subscrive to input video feed and publish output video feed
        imageSub = it.subscribeCamera("/depth_camera/image_raw", 1, &PoseDetector::imageCallBack, this);
        imagePub = it.advertise("/image_converter/output_video", 1);
        posePub = nh.advertise<geometry_msgs::Pose>("/tag_pose", 1);

        cv::namedWindow(OPENCV_WINDOW);

        td = apriltag_detector_create();
        tf = tag36h11_create();
        apriltag_detector_add_family(td, tf);

        info.tagsize = 0.5;
        info.fx = 1.047;
        info.fy = 1.047;
        info.cx = 0;
        info.cy = 0;

        T_WC << 1, 0, 0, -2,
                0, 1, 0, 0,
                0, 0, 1, 1,
                0, 0, 0, 1;
    }

    ~PoseDetector()
    {
        cv::destroyWindow(OPENCV_WINDOW);
        tag36h11_destroy(tf);
        apriltag_detector_destroy(td);
    }

    void imageCallBack(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::CameraInfoConstPtr &camera_info)
    {
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

        // // Draw an example circle on the video stream
        // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
        //     cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255, 0, 0));

        image_u8_t img_header = {.width = cv_ptr->image.cols,
                                 .height = cv_ptr->image.rows,
                                 .stride = cv_ptr->image.cols,
                                 .buf = cv_ptr->image.data};

        zarray_t *detections = apriltag_detector_detect(td, &img_header);
        // std::cout << zarray_size(detections) << " tags detected" << std::endl;

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

            // First create an apriltag_detection_info_t struct using your known parameters.
            info.det = det;
            double err = estimate_tag_pose(&info, &pose);

            Eigen::Matrix4d T_CT;
            T_CT << pose.R->data[0], pose.R->data[1], pose.R->data[2], pose.t->data[0],
                    pose.R->data[3], pose.R->data[4], pose.R->data[5], pose.t->data[1],
                    pose.R->data[6], pose.R->data[7], pose.R->data[8], pose.t->data[2],
                    0, 0, 0, 1;

            T_WT = T_WC * T_CT;

            geometry_msgs::Pose currentPose;
            tf::Quaternion qt;

            currentPose.position.x = T_WT(0, 3);
            currentPose.position.y = T_WT(1, 3);
            currentPose.position.z = T_WT(2, 3);

            tf::Matrix3x3 rotationMatrix(T_WT(0, 0), T_WT(0, 1), T_WT(0, 2),
                                         T_WT(1, 0), T_WT(1, 1), T_WT(1, 2),
                                         T_WT(2, 0), T_WT(2, 1), T_WT(2, 2));

            rotationMatrix.getRotation(qt);
            tf::quaternionTFToMsg(qt, currentPose.orientation);

            // transformStuff
            tf::StampedTransform transform;

            // debug the position info
            // std::cout << currentPose.position.x << " "
            //           << currentPose.position.y << " "
            //           << currentPose.position.z << " "
            //           << currentPose.orientation.x << " "
            //           << currentPose.orientation.y << " "
            //           << currentPose.orientation.z << " "
            //           << currentPose.orientation.w << " "
            //           << std::endl;

            posePub.publish(currentPose);
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

private:
    ros::NodeHandle nh;
    ros::Publisher posePub;
    image_transport::ImageTransport it;
    image_transport::CameraSubscriber imageSub;
    image_transport::Publisher imagePub;
    // tf::TransformListener listener;

    apriltag_detector_t *td;
    apriltag_family_t *tf;
    apriltag_pose_t pose;
    apriltag_detection_info_t info;

    Eigen::Matrix4d T_WC;
    Eigen::Matrix4d T_WT;

    // apriltag_ros::TagDetector tagDetector;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_converter");
    PoseDetector ic;
    ros::spin();
    return 0;
}
