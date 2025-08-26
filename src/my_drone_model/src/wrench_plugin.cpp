#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <ignition/math/Vector3.hh>

namespace gazebo {
class QuadrotorWrenchPlugin : public ModelPlugin {
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr /*_sdf*/) {
    // Initialize ROS node handle once
    if (!ros::isInitialized()) {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "quadrotor_wrench_plugin",
                ros::init_options::NoSigintHandler);
    }

    this->link = model->GetLink("frame");
    if (!this->link) {
      gzerr << "[QuadrotorWrenchPlugin] Link named 'frame' not found!\n";
      return;
    }

    this->nh.reset(new ros::NodeHandle("~"));
    this->sub = nh->subscribe("/quadrotor/thrust_cmd", 1,
                              &QuadrotorWrenchPlugin::ApplyWrench, this);
  }

  void ApplyWrench(const geometry_msgs::Wrench::ConstPtr& msg) {
    ignition::math::Vector3d force(msg->force.x, msg->force.y, msg->force.z);
    ignition::math::Vector3d torque(msg->torque.x, msg->torque.y, msg->torque.z);

    this->link->AddRelativeForce(force);     // LOCAL/BODY frame
    this->link->AddRelativeTorque(torque);   // LOCAL/BODY frame
  }

private:
  physics::LinkPtr link;
  std::unique_ptr<ros::NodeHandle> nh;
  ros::Subscriber sub;
};

GZ_REGISTER_MODEL_PLUGIN(QuadrotorWrenchPlugin)
}  // namespace gazebo
