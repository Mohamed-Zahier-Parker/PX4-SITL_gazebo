
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

  //Subscribe to state machine state topic
  sm_state_sub_topic_="/state_machine_state";
  sm_state_sub_ = node_handle_->Subscribe<mp_msgs::msgs::StateMachineState>("~/" + sm_state_sub_topic_, &MovingPlatformPlugin::StateMachineStateCallback, this);

  //Advertise publishing to moving platform topic
  moving_platform_pub_= node_handle_->Advertise<mp_msgs::msgs::MovingPlatform>("~/" + moving_platform_pub_topic_, 1);
}

/////////////////////////////////////////////////
void MovingPlatformPlugin::OnUpdate()
{
double offset_plat[3]={0.0,0.0,0.0};//offset of uav spawn point with gazebo origin
//   std::cout<<"Running MovingPlatfromPlugin\n";
  GZ_ASSERT(this->link, "Link was NULL");

  dT_defl_=dT_defl_/3500.00*40.00;

  // std::cout<<"Thrust value : "<<dT_defl_<<"\n";

  //Get platform position
    // pose of body
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3d pose = this->link->WorldPose();
#else
  ignition::math::Pose3d pose = ignitionFromGazeboMath(this->link->GetWorldPose());
#endif

  // Get platform velocity
  ignition::math::Vector3d vel = this->link->WorldLinearVel();


  //  int sm_state_out = 0;//For testing

  //Platform velocities
  ignition::math::Vector3d mp_initial_velocity=ignition::math::Vector3d(1.00,0.00,0.00);// Velocity to reach starting point
  ignition::math::Vector3d mp_normal_velocity=ignition::math::Vector3d(10.00,0.00,0.00);// Velocity to move with
  ignition::math::Vector3d mp_return_velocity=ignition::math::Vector3d(-10.00,0.00,0.00);// Velocity to return to starting point

  //move to start position
  if(dT_defl_ > 0.00 && (pose.X()-offset_plat[0])<=0.0 && sm_state_out==0 && !start_initial_mp_movement && !end_initial_mp_movement){
    start_initial_mp_movement=true;
    std::cout<<"Move platform to start position\n";
    this->link->AddRelativeForce({100,0,0});//Need initial force to get it moving due to ground plane having static friction
  }else if(start_initial_mp_movement && (pose.X()-offset_plat[0])<0.0){
    this->link->SetLinearVel(mp_initial_velocity);
  }else if(start_initial_mp_movement && (pose.X()-offset_plat[0])>=0.0)
  {
    this->link->SetLinearVel({0, 0, 0});
    start_initial_mp_movement=false;
    end_initial_mp_movement=true;
  }

  // Move platform for FW UAV landing
  if(end_initial_mp_movement){
    if(sm_state_out==0 && (pose.X()-offset_plat[0])>0.1 && !end_state_mp_movement){ //Check Abort case
      this->link->SetLinearVel(mp_return_velocity);
      start_state_mp_movement=false;
    }else if(sm_state_out>=1 && !start_state_mp_movement && !end_state_mp_movement){
      start_state_mp_movement=true;
      std::cout<<"Landing platform moving\n";
      this->link->AddRelativeForce({100,0,0});//Need initial force to get it moving due to ground plane having static friction
    }else if(start_state_mp_movement && sm_state_out!=6 && !end_state_mp_movement){
	    this->link->SetLinearVel(mp_normal_velocity);
    }else if(sm_state_out==6 && !end_state_mp_movement && !start_state_mp_movement){
      this->link->SetLinearVel(ignition::math::Vector3(0.00,0.00,0.00));
      end_state_mp_movement=true;
      std::cout<<"Gazebo_comp_mp_x : "<<pose.Y()<<" Gazebo_comp_mp_y : "<<pose.X()<<" Gazebo_comp_mp_z : "<<(-pose.Z()-0.05)<<"\n";
    }else{
	    this->link->SetLinearVel(ignition::math::Vector3(0.00,0.00,0.00));
    }
  }

  //Prepare data to be published (Convert to NED frame from ENU)
  ignition::math::Vector3d mp_pose_pub=ignition::math::Vector3d((pose.Y()-offset_plat[1]),(pose.X()-offset_plat[0]),-(pose.Z()+0.05)); // 0.05 added due to platform being placed 0.05m above ground
  ignition::math::Vector3d mp_vel_pub=ignition::math::Vector3d(vel.Y(),vel.X(),-vel.Z());

  // mp_pose_pub=gazebo::msgs::Convert(mp_pose_pub);
  // mp_vel_pub=gazebo::msgs::Convert(mp_vel_pub);

  // Publish moving platform stats to topic
  mp_msgs::msgs::MovingPlatform mp_stats_msg;

  mp_stats_msg.set_mp_position_x(mp_pose_pub.X());
  mp_stats_msg.set_mp_position_y(mp_pose_pub.Y());
  mp_stats_msg.set_mp_position_z(mp_pose_pub.Z());
  mp_stats_msg.set_mp_velocity_x(mp_vel_pub.X());
  mp_stats_msg.set_mp_velocity_y(mp_vel_pub.Y());
  mp_stats_msg.set_mp_velocity_z(mp_vel_pub.Z());

  if(mp_pub_count%50==0){ //Temporary fix for too fast publishing
    moving_platform_pub_->Publish(mp_stats_msg);
  }
  mp_pub_count++;
}

void MovingPlatformPlugin::ActuatorDeflectionCallback(ActuatorDeflectionsPtr &deflections) {
  dA_defl_=deflections->da();
  dE_defl_=deflections->de();
  dF_defl_=deflections->df();
  dR_defl_=deflections->dr();
  dT_defl_=deflections->dt();
  // std::cout<<"Running actuator deflections dT = "<<dT_defl_<<"\n";
}

void MovingPlatformPlugin::StateMachineStateCallback(StateMachineStatePtr &sm_msg){
  sm_state_out=sm_msg->sm_state();
}
