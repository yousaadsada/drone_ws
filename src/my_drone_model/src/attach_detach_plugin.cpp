#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>

namespace gazebo
{
class BallAttachPlugin : public ModelPlugin
{
public:
  BallAttachPlugin() {}

  void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    this->model = _parent;
    this->world = this->model->GetWorld();
    this->link1 = this->model->GetLink("ball");
    this->link2 = this->model->GetLink("link");

    if (!ros::isInitialized())
    {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "ball_attach_plugin_node");
    }

    this->rosNode.reset(new ros::NodeHandle("ball_attach_plugin"));

    // Service to attach or detach the ball
    this->srv = this->rosNode->advertiseService("attach_ball", &BallAttachPlugin::OnAttachRequest, this);
  }

  bool OnAttachRequest(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
  {
    if (req.data)  // Attach
    {
      if (!this->joint)
      {
        this->joint = this->world->Physics()->CreateJoint("fixed", this->model);
        this->joint->Attach(this->link1, this->link2);
        this->joint->Load(this->link1, this->link2, ignition::math::Pose3d());
        this->joint->Init();
        this->joint->SetName(this->model->GetName() + "_fixed_joint");
        res.success = true;
        res.message = "Ball attached.";
      }
      else
      {
        res.success = false;
        res.message = "Already attached.";
      }
    }
    else  // Detach
    {
      if (this->joint)
      {
        this->joint->Detach();
        this->joint.reset();
        res.success = true;
        res.message = "Ball detached.";
      }
      else
      {
        res.success = false;
        res.message = "Already detached.";
      }
    }
    return true;
  }

private:
  physics::ModelPtr model;
  physics::WorldPtr world;
  physics::LinkPtr link1, link2;
  physics::JointPtr joint;

  std::unique_ptr<ros::NodeHandle> rosNode;
  ros::ServiceServer srv;
};
GZ_REGISTER_MODEL_PLUGIN(BallAttachPlugin)
}
