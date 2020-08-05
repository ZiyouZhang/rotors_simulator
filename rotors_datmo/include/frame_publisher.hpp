#ifndef FRAME_PUBLISHER_H
#define FRAME_PUBLISHER_H

#include <Eigen/Core>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>    
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
  class FramePublisherPlugin : public WorldPlugin
  {
  public:
    FramePublisherPlugin();
    ~FramePublisherPlugin();

    void InitializeParams();
    void Publish();

  protected:
    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);
    void OnUpdate(const common::UpdateInfo &);
  };
} // namespace gazebo

#endif // FRAME_PUBLISHER_H
