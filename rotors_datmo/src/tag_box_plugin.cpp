#include "tag_box_plugin.hpp"

#include <ros/console.h>
#include <geometry_msgs/Pose.h>

namespace gazebo
{
    void TagBoxPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        // Store the pointer to the model
        model = _parent;
        pose = model->WorldPose();
        msg.header.frame_id = model->GetName();

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&TagBoxPlugin::OnUpdate, this));

        posePub = nh.advertise<nav_msgs::Odometry>("/tag_box_pose_ground_truth", 1);
    }

    void TagBoxPlugin::OnUpdate()
    {
        msg.header.stamp = ros::Time::now();
        // Apply a small linear velocity to the model.
        if (model->WorldPose().Pos().Z() < 0.3)
            model->SetLinearVel(ignition::math::Vector3d(0.001, 0.001, 4));

        // model->SetLinearVel(ignition::math::Vector3d(0.05, 0.05, 0.001));

        // Publish pose message
        msg.pose.pose.position.x = model->WorldPose().Pos().X();
        msg.pose.pose.position.y = model->WorldPose().Pos().Y();
        msg.pose.pose.position.z = model->WorldPose().Pos().Z();

        msg.twist.twist.linear.x = model->RelativeLinearVel()[0];
        msg.twist.twist.linear.y = model->RelativeLinearVel()[1];
        msg.twist.twist.linear.z = model->RelativeLinearVel()[2];

        msg.twist.twist.angular.x = model->RelativeAngularVel()[0];
        msg.twist.twist.angular.y = model->RelativeAngularVel()[1];
        msg.twist.twist.angular.z = model->RelativeAngularVel()[2];

        posePub.publish(msg);

        // Broadcast tf
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(model->WorldPose().Pos().X(),
                                        model->WorldPose().Pos().Y(),
                                        model->WorldPose().Pos().Z()));
        tf::Quaternion quaternion;
        quaternion.setRPY(model->WorldPose().Rot().Roll(),
                          model->WorldPose().Rot().Pitch(),
                          model->WorldPose().Rot().Yaw());
        transform.setRotation(quaternion);

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", model->GetName()));
    }

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(TagBoxPlugin)
} // namespace gazebo