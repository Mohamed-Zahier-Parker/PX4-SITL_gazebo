
#include <algorithm>
#include <string>

#include "common.h"
#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "moving_platform_plugin.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(MovingPlatformPlugin)

MovingPlatformPlugin::MovingPlatformPlugin()
{

}

MovingPlatformPlugin::~MovingPlatformPlugin()
{
}

void MovingPlatformPlugin::Load(physics::ModelPtr _model,
                     sdf::ElementPtr _sdf)
{

GZ_ASSERT(_model, "MovingPlatformPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "MovingPlatformPlugin _sdf pointer is NULL");
  this->model = _model;
  this->sdf = _sdf;

  this->world = this->model->GetWorld();
  GZ_ASSERT(this->world, "MovingPlatformPlugin world pointer is NULL");

#if GAZEBO_MAJOR_VERSION >= 9
  this->physics = this->world->Physics();
#else
  this->physics = this->world->GetPhysicsEngine();
#endif
  GZ_ASSERT(this->physics, "MovingPlatformPlugin physics pointer is NULL");

  GZ_ASSERT(_sdf, "MovingPlatformPlugin _sdf pointer is NULL");

  if (_sdf->HasElement("link_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("link_name");
    // GZ_ASSERT(elem, "Element link_name doesn't exist!");
    std::string linkName = elem->Get<std::string>();
    this->link = this->model->GetLink(linkName);
    // GZ_ASSERT(this->link, "Link was NULL");

    if (!this->link)
    {
      gzerr << "Link with name[" << linkName << "] not found. "
        << "The MovingPlatformPlugin will not generate forces\n";
    }
    else
    {
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&MovingPlatformPlugin::OnUpdate, this));
    }


  }

  if (_sdf->HasElement("UAV_model_name"))
  {
    uav_model_name = _sdf->GetElement("UAV_model_name")->Get<std::string>();
  } else {
    gzerr << "Please specify a UAV_model_name.\n";
  }

  if (_sdf->HasElement("robotNamespace"))
  {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_wind_plugin] Please specify a robotNamespace.\n";
  }
  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  //Subscribe to actuator deflection topic
  act_def_sub_topic_="/actuator_deflections";
  act_def_sub_ = node_handle_->Subscribe<act_msgs::msgs::ActuatorDeflections>("~/" + uav_model_name + act_def_sub_topic_, &MovingPlatformPlugin::ActuatorDeflectionCallback, this);



}

/////////////////////////////////////////////////
void MovingPlatformPlugin::OnUpdate()
{

//   std::cout<<"Running MovingPlatfromPlugin\n";
  GZ_ASSERT(this->link, "Link was NULL");

  dT_defl_=dT_defl_/3500.00*40.00;

//   std::cout<<"Thrust value : "<<dT_defl_<<"\n";

  //Get platform position
    // pose of body
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3d pose = this->link->WorldPose();
#else
  ignition::math::Pose3d pose = ignitionFromGazeboMath(this->link->GetWorldPose());
#endif


   int state = 0;//For testing

  //Platform velocities
  ignition::math::Vector3 mp_initial_velocity=ignition::math::Vector3(1.00,0.00,0.00);
  ignition::math::Vector3 mp_normal_velocity=ignition::math::Vector3(10.00,0.00,0.00);
  ignition::math::Vector3 mp_return_velocity=ignition::math::Vector3(-10.00,0.00,0.00);

  if(dT_defl_ > 0 && pose.X()<=-0.1 && state==0){ //move to start position
	  this->link->SetLinearVel(mp_initial_velocity);
  }else if(state>=1){
	  this->link->SetLinearVel(mp_normal_velocity);
  }else if(state==0 && pose.X()>0){
	  this->link->SetLinearVel(mp_return_velocity);
  }else{
	  this->link->SetLinearVel(ignition::math::Vector3(0.00,0.00,0.00));
  }

}

void MovingPlatformPlugin::ActuatorDeflectionCallback(ActuatorDeflectionsPtr &deflections) {
  dA_defl_=deflections->da();
  dE_defl_=deflections->de();
  dF_defl_=deflections->df();
  dR_defl_=deflections->dr();
  dT_defl_=deflections->dt();
}
