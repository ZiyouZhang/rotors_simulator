#ifndef TAG_BOX_PLUGIN_H
#define TAG_BOX_PLUGIN_H

#include <ros/ros.h>

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <nav_msgs/Odometry.h>

#include <tf/transform_broadcaster.h>

namespace gazebo
{
    class TagBoxPlugin : public ModelPlugin
    {
    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr);
        // void PoseCallback(const geometry_msgs::PoseConstPtr &msg);

        // Called by the world update start event
        void OnUpdate();

    private:
        // Pointer to the model
        physics::ModelPtr model;

        // Pointer to the update event connection
        event::ConnectionPtr updateConnection;

        // Navigation pose message
        nav_msgs::Odometry msg;

        // Pose message
        ignition::math::Pose3d pose;

        ros::NodeHandle nh;
        ros::Publisher posePub;

        ros::Subscriber frameSub;
        tf::TransformBroadcaster br;
    };

} // namespace gazebo

#endif // TAG_BOX_PLUGIN_H