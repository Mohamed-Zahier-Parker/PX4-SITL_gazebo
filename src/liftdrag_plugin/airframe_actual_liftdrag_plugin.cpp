/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <algorithm>
#include <string>

#include "common.h"
#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "liftdrag_plugin/airframe_actual_liftdrag_plugin.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(AirframeActualLiftDragPlugin)

/////////////////////////////////////////////////
AirframeActualLiftDragPlugin::AirframeActualLiftDragPlugin() : cla(1.0), cda(0.01), cma(0.0), rho(1.2041)
{
  this->cp = ignition::math::Vector3d(0, 0, 0);
  this->forward = ignition::math::Vector3d(1, 0, 0);
  this->upward = ignition::math::Vector3d(0, 0, 1);
  this->wind_vel_ = ignition::math::Vector3d(0.0, 0.0, 0.0);
  this->area = 1.0;
  this->alpha0 = 0.0;
  this->alpha = 0.0;
  this->sweep = 0.0;
  this->velocityStall = 0.0;

  //single airframe constants
  this->S=0.0;
  this->b=0.0;
  this->cbar=0.0;
  this->A=0.0;
  this->e=0.0;
  this->beta =0.0;

  // 90 deg stall
  this->alphaStall = 0.5*M_PI;
  this->claStall = 0.0;

  this->radialSymmetry = false;

  /// \TODO: what's flat plate drag?
  this->cdaStall = 1.0;
  this->cmaStall = 0.0;

  /// how much to change CL per every radian of the control joint value
  this->controlJointRadToCL = 4.0;

  // How much Cm changes with a change in control surface deflection angle
  this->cm_delta = 0.0;

  //single aircraft coefficients
  this->C_L_0 = 0;this->C_L_alpha = 0;this->C_L_Q = 0;this->C_L_dE = 0;this->C_L_dF = 0;
  this->C_D_0 =0;
  this->C_y_beta=0;this->C_y_P=0;this->C_y_R=0;this->C_y_dA=0;this->C_y_dR=0;
  this->C_l_beta=0;this->C_l_P=0;this->C_l_R=0;this->C_l_dA=0;this->C_l_dR=0;
  this->C_n_beta=0;this->C_n_P=0;this->C_n_R=0;this->C_n_dA=0;this->C_n_dR=0;
  this->C_m_0 = 0;this->C_m_alpha = 0;this->C_m_Q = 0;this->C_m_dE = 0;this->C_m_dF = 0;

}

/////////////////////////////////////////////////
AirframeActualLiftDragPlugin::~AirframeActualLiftDragPlugin()
{
}

/////////////////////////////////////////////////
void AirframeActualLiftDragPlugin::Load(physics::ModelPtr _model,
                     sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "AirframeActualLiftDragPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "AirframeActualLiftDragPlugin _sdf pointer is NULL");
  this->model = _model;
  this->sdf = _sdf;

  this->world = this->model->GetWorld();
  GZ_ASSERT(this->world, "AirframeActualLiftDragPlugin world pointer is NULL");

#if GAZEBO_MAJOR_VERSION >= 9
  this->physics = this->world->Physics();
#else
  this->physics = this->world->GetPhysicsEngine();
#endif
  GZ_ASSERT(this->physics, "AirframeActualLiftDragPlugin physics pointer is NULL");

  GZ_ASSERT(_sdf, "AirframeActualLiftDragPlugin _sdf pointer is NULL");

  if (_sdf->HasElement("radial_symmetry"))
    this->radialSymmetry = _sdf->Get<bool>("radial_symmetry");

  if (_sdf->HasElement("a0"))
    this->alpha0 = _sdf->Get<double>("a0");

  if (_sdf->HasElement("cla"))
    this->cla = _sdf->Get<double>("cla");

  if (_sdf->HasElement("cda"))
    this->cda = _sdf->Get<double>("cda");

  if (_sdf->HasElement("cma"))
    this->cma = _sdf->Get<double>("cma");

  if (_sdf->HasElement("alpha_stall"))
    this->alphaStall = _sdf->Get<double>("alpha_stall");

  if (_sdf->HasElement("cla_stall"))
    this->claStall = _sdf->Get<double>("cla_stall");

  if (_sdf->HasElement("cda_stall"))
    this->cdaStall = _sdf->Get<double>("cda_stall");

  if (_sdf->HasElement("cma_stall"))
    this->cmaStall = _sdf->Get<double>("cma_stall");

    if (_sdf->HasElement("cm_delta"))
        this->cm_delta = _sdf->Get<double>("cm_delta");

  if (_sdf->HasElement("cp"))
    this->cp = _sdf->Get<ignition::math::Vector3d>("cp");

  // blade forward (-drag) direction in link frame
  if (_sdf->HasElement("forward"))
    this->forward = _sdf->Get<ignition::math::Vector3d>("forward");
  this->forward.Normalize();

  // blade upward (+lift) direction in link frame
  if (_sdf->HasElement("upward"))
    this->upward = _sdf->Get<ignition::math::Vector3d>("upward");
  this->upward.Normalize();

  if (_sdf->HasElement("area"))
    this->area = _sdf->Get<double>("area");

  if (_sdf->HasElement("air_density"))
    this->rho = _sdf->Get<double>("air_density");

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
        << "The AirframeActualLiftDragPlugin will not generate forces\n";
    }
    else
    {
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&AirframeActualLiftDragPlugin::OnUpdate, this));
    }


  }

  if (_sdf->HasElement("robotNamespace"))
  {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_wind_plugin] Please specify a robotNamespace.\n";
  }
  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  if (_sdf->HasElement("windSubTopic")){
    this->wind_sub_topic_ = _sdf->Get<std::string>("windSubTopic");
    //wind_sub_ = node_handle_->Subscribe("~/" + wind_sub_topic_, &AirframeActualLiftDragPlugin::WindVelocityCallback, this);
  }

  //Subscribe to actuator deflection topic
  act_def_sub_topic_="/actuator_deflections";
  act_def_sub_ = node_handle_->Subscribe<act_msgs::msgs::ActuatorDeflections>("~/" + model->GetName() + act_def_sub_topic_, &AirframeActualLiftDragPlugin::ActuatorDeflectionCallback, this);

  //Subscribe to world stats topic
  world_stats_sub_ = node_handle_->Subscribe("~/" + world_stats_sub_topic_, &AirframeActualLiftDragPlugin::WorldStatsCallback, this);


  if (_sdf->HasElement("control_joint_name"))
  {
    std::string controlJointName = _sdf->Get<std::string>("control_joint_name");
    this->controlJoint = this->model->GetJoint(controlJointName);
    if (!this->controlJoint)
    {
      gzerr << "Joint with name[" << controlJointName << "] does not exist.\n";
    }
  }

    if (_sdf->HasElement("control_joint_name_dA"))
  {
    std::string controlJointNamedA = _sdf->Get<std::string>("control_joint_name_dA");
    this->controlJointdA = this->model->GetJoint(controlJointNamedA);
    if (!this->controlJointdA)
    {
      gzerr << "Joint with name[" << controlJointNamedA << "] does not exist.\n";
    }
  }

    if (_sdf->HasElement("control_joint_name_dE"))
  {
    std::string controlJointNamedE = _sdf->Get<std::string>("control_joint_name_dE");
    this->controlJointdE = this->model->GetJoint(controlJointNamedE);
    if (!this->controlJointdE)
    {
      gzerr << "Joint with name[" << controlJointNamedE << "] does not exist.\n";
    }
  }

    if (_sdf->HasElement("control_joint_name_dR"))
  {
    std::string controlJointNamedR = _sdf->Get<std::string>("control_joint_name_dR");
    this->controlJointdR = this->model->GetJoint(controlJointNamedR);
    if (!this->controlJointdR)
    {
      gzerr << "Joint with name[" << controlJointNamedR << "] does not exist.\n";
    }
  }

    if (_sdf->HasElement("control_joint_name_dF"))
  {
    std::string controlJointNamedF = _sdf->Get<std::string>("control_joint_name_dF");
    this->controlJointdF = this->model->GetJoint(controlJointNamedF);
    if (!this->controlJointdF)
    {
      gzerr << "Joint with name[" << controlJointNamedF << "] does not exist.\n";
    }
  }

  if (_sdf->HasElement("control_joint_rad_to_cl"))
    this->controlJointRadToCL = _sdf->Get<double>("control_joint_rad_to_cl");

  if (_sdf->HasElement("S"))
    this->S = _sdf->Get<double>("S");
  if (_sdf->HasElement("b"))
    this->b = _sdf->Get<double>("b");
  if (_sdf->HasElement("cbar"))
    this->cbar = _sdf->Get<double>("cbar");
  if (_sdf->HasElement("A"))
    this->A = _sdf->Get<double>("A");
  if (_sdf->HasElement("e"))
    this->e = _sdf->Get<double>("e");

  if (_sdf->HasElement("C_L_0"))
    this->C_L_0 = _sdf->Get<double>("C_L_0");
  if (_sdf->HasElement("C_L_alpha"))
    this->C_L_alpha = _sdf->Get<double>("C_L_alpha");
  if (_sdf->HasElement("C_L_Q"))
    this->C_L_Q = _sdf->Get<double>("C_L_Q");
  if (_sdf->HasElement("C_L_dE"))
    this->C_L_dE = _sdf->Get<double>("C_L_dE");
  if (_sdf->HasElement("C_L_dF"))
    this->C_L_dF = _sdf->Get<double>("C_L_dF");
  if (_sdf->HasElement("C_D_0"))
    this->C_D_0 = _sdf->Get<double>("C_D_0");
  if (_sdf->HasElement("C_y_beta"))
    this->C_y_beta = _sdf->Get<double>("C_y_beta");
  if (_sdf->HasElement("C_y_P"))
    this->C_y_P = _sdf->Get<double>("C_y_P");
  if (_sdf->HasElement("C_y_R"))
    this->C_y_R = _sdf->Get<double>("C_y_R");
  if (_sdf->HasElement("C_y_dA"))
    this->C_y_dA = _sdf->Get<double>("C_y_dA");
  if (_sdf->HasElement("C_y_dR"))
    this->C_y_dR = _sdf->Get<double>("C_y_dR");
  if (_sdf->HasElement("C_l_beta"))
    this->C_l_beta = _sdf->Get<double>("C_l_beta");
  if (_sdf->HasElement("C_l_P"))
    this->C_l_P = _sdf->Get<double>("C_l_P");
  if (_sdf->HasElement("C_l_R"))
    this->C_l_R = _sdf->Get<double>("C_l_R");
  if (_sdf->HasElement("C_l_dA"))
    this->C_l_dA = _sdf->Get<double>("C_l_dA");
  if (_sdf->HasElement("C_l_dR"))
    this->C_l_dR = _sdf->Get<double>("C_l_dR");
  if (_sdf->HasElement("C_n_beta"))
    this->C_n_beta = _sdf->Get<double>("C_n_beta");
  if (_sdf->HasElement("C_n_P"))
    this->C_n_P = _sdf->Get<double>("C_n_P");
  if (_sdf->HasElement("C_n_R"))
    this->C_n_R = _sdf->Get<double>("C_n_R");
  if (_sdf->HasElement("C_n_dA"))
    this->C_n_dA = _sdf->Get<double>("C_n_dA");
  if (_sdf->HasElement("C_n_dR"))
    this->C_n_dR = _sdf->Get<double>("C_n_dR");
  if (_sdf->HasElement("C_m_0"))
    this->C_m_0 = _sdf->Get<double>("C_m_0");
  if (_sdf->HasElement("C_m_alpha"))
    this->C_m_alpha = _sdf->Get<double>("C_m_alpha");
  if (_sdf->HasElement("C_m_Q"))
    this->C_m_Q = _sdf->Get<double>("C_m_Q");
  if (_sdf->HasElement("C_m_dE"))
    this->C_m_dE = _sdf->Get<double>("C_m_dE");
  if (_sdf->HasElement("C_m_dF"))
    this->C_m_dF = _sdf->Get<double>("C_m_dF");

  if (_sdf->HasElement("V_20"))
    this->V_20 = _sdf->Get<double>("V_20");
  // if (_sdf->HasElement("Vm"))
  //   this->Vm = _sdf->Get<ignition::math::Vector3d>("Vm");
  // if (_sdf->HasElement("ts"))
  //   this->ts = _sdf->Get<ignition::math::Vector3d>("ts");
  // if (_sdf->HasElement("dm"))
  //   this->dm = _sdf->Get<ignition::math::Vector3d>("dm");
  // if (_sdf->HasElement("ds"))
  if (_sdf->HasElement("Vm"))
    this->Vm = _sdf->Get<double>("Vm");
  if (_sdf->HasElement("ts"))
    this->ts = _sdf->Get<double>("ts");
  if (_sdf->HasElement("dm"))
    this->dm = _sdf->Get<double>("dm");
  if (_sdf->HasElement("ds"))
    this->ds = _sdf->Get<double>("ds");
  if (_sdf->HasElement("gust_direction"))
    this->gust_direction = _sdf->Get<ignition::math::Vector3d>("gust_direction");

  if (_sdf->HasElement("V_20_shear"))
    this->V_20_shear = _sdf->Get<double>("V_20_shear");
  if (_sdf->HasElement("shear_direction"))
    this->shear_direction = _sdf->Get<ignition::math::Vector3d>("shear_direction");

  wind_pub_ = node_handle_->Advertise<physics_msgs::msgs::Wind>("~/" + wind_pub_topic_, 10);

}

/////////////////////////////////////////////////
void AirframeActualLiftDragPlugin::OnUpdate()
{
  // std::cout<<"Testing AirframeActualLiftDragPlugin\n";
  GZ_ASSERT(this->link, "Link was NULL");
  // get linear velocity at cp in inertial frame
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d vel = this->link->WorldLinearVel(this->cp) - wind_vel_ - custom_wind_lin_vel_;
#else
  ignition::math::Vector3d vel = ignitionFromGazeboMath(this->link->GetWorldLinearVel(this->cp)) - wind_vel_ - custom_wind_lin_vel_;
#endif
  ignition::math::Vector3d velI = vel;
  velI.Normalize();

  if (vel.Length() <= 0.01)
    return;

  // pose of body
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3d pose = this->link->WorldPose();
#else
  ignition::math::Pose3d pose = ignitionFromGazeboMath(this->link->GetWorldPose());
#endif

  // rotate forward and upward vectors into inertial frame
  ignition::math::Vector3d forwardI = pose.Rot().RotateVector(this->forward);

  if (forwardI.Dot(vel) <= 0.0){
    // Only calculate lift or drag if the wind relative velocity is in the same direction
    return;
  }

  ignition::math::Vector3d upwardI;
  if (this->radialSymmetry)
  {
    // use inflow velocity to determine upward direction
    // which is the component of inflow perpendicular to forward direction.
    ignition::math::Vector3d tmp = forwardI.Cross(velI);
    upwardI = forwardI.Cross(tmp).Normalize();
  }
  else
  {
    upwardI = pose.Rot().RotateVector(this->upward);
  }

  // convert to aircraft axes
  forwardI=ignition::math::Vector3d(forwardI.X(),-forwardI.Y(),-forwardI.Z());
  upwardI=ignition::math::Vector3d(upwardI.X(),-upwardI.Y(),-upwardI.Z());
  vel=ignition::math::Vector3d(vel.X(),-vel.Y(),-vel.Z());

  // spanwiseI: a vector normal to lift-drag-plane described in inertial frame
  ignition::math::Vector3d spanwiseI = forwardI.Cross(upwardI).Normalize();

  const double minRatio = -1.0;
  const double maxRatio = 1.0;
  // check sweep (angle between velI and lift-drag-plane)
  double sinSweepAngle = ignition::math::clamp(
      spanwiseI.Dot(velI), minRatio, maxRatio);

  this->sweep = asin(sinSweepAngle);

  // truncate sweep to within +/-90 deg
  while (fabs(this->sweep) > 0.5 * M_PI)
    this->sweep = this->sweep > 0 ? this->sweep - M_PI
                                  : this->sweep + M_PI;
  // get cos from trig identity
  double cosSweepAngle = sqrt(1.0 - sin(this->sweep) * sin(this->sweep));

  // angle of attack is the angle between
  // velI projected into lift-drag plane
  //  and
  // forward vector
  //
  // projected = spanwiseI Xcross ( vector Xcross spanwiseI)
  //
  // so,
  // removing spanwise velocity from vel
  ignition::math::Vector3d velInLDPlane = vel - vel.Dot(spanwiseI)*spanwiseI;

  // get direction of drag
  ignition::math::Vector3d dragDirection = -velInLDPlane;
  dragDirection.Normalize();

  // get direction of lift
  ignition::math::Vector3d liftI = spanwiseI.Cross(velInLDPlane);
  liftI.Normalize();

  // get direction of moment
  ignition::math::Vector3d momentDirection = spanwiseI;

  // get direction of lateral force !!!CHECK!!!
  ignition::math::Vector3d velInSDPlane = vel - vel.Dot(upwardI)*upwardI;
  ignition::math::Vector3d slipDirection = velInSDPlane.Cross(liftI);
  slipDirection.Normalize();
  //workout beta !!!Check!!!
  double cosBeta=ignition::math::clamp(slipDirection.Dot(spanwiseI), -1.0, 1.0);
  if (slipDirection.Dot(forwardI) <= 0.0) //!!!CHECK direction of beta!!!
    this->beta = 0+ acos(cosBeta);
  else
    this->beta = 0-acos(cosBeta);

    // normalize to within +/-90 deg
  while (fabs(this->beta) > 0.5 * M_PI)
    this->beta = this->beta > 0 ? this->beta - M_PI
                                  : this->beta + M_PI;

  // compute angle between upwardI and liftI
  // in general, given vectors a and b:
  //   cos(theta) = a.Dot(b)/(a.Length()*b.Lenghth())
  // given upwardI and liftI are both unit vectors, we can drop the denominator
  //   cos(theta) = a.Dot(b)
  double cosAlpha = ignition::math::clamp(liftI.Dot(upwardI), minRatio, maxRatio);

  // Is alpha positive or negative? Test:
  // forwardI points toward zero alpha
  // if forwardI is in the same direction as lift, alpha is positive.
  // liftI is in the same direction as forwardI?
  if (liftI.Dot(forwardI) >= 0.0)
    this->alpha = this->alpha0 + acos(cosAlpha);
  else
    this->alpha = this->alpha0 - acos(cosAlpha);

  // normalize to within +/-90 deg
  while (fabs(this->alpha) > 0.5 * M_PI)
    this->alpha = this->alpha > 0 ? this->alpha - M_PI
                                  : this->alpha + M_PI;

  // compute dynamic pressure
  double speedInLDPlane = velInLDPlane.Length();
  double total_speed=vel.Length();
  // double q = 0.5 * this->rho * speedInLDPlane * speedInLDPlane;
  double q = 0.5 * this->rho * total_speed * total_speed;

  // compute cl at cp, check for stall, correct for sweep
  double cl;
  if (this->alpha > this->alphaStall)
  {
    cl = (this->cla * this->alphaStall +
          this->claStall * (this->alpha - this->alphaStall))
         * cosSweepAngle;
    // make sure cl is still great than 0
    cl = std::max(0.0, cl);
  }
  else if (this->alpha < -this->alphaStall)
  {
    cl = (-this->cla * this->alphaStall +
          this->claStall * (this->alpha + this->alphaStall))
         * cosSweepAngle;
    // make sure cl is still less than 0
    cl = std::min(0.0, cl);
  }
  else
    cl = this->cla * this->alpha * cosSweepAngle;

  // modify cl per control joint value
  double controlAngle;
  double dA=0;
  double dE=0;
  double dR=0;
  double dF=0;
  if (this->controlJoint)
  {
#if GAZEBO_MAJOR_VERSION >= 9
    controlAngle = this->controlJoint->Position(0);
#else
    controlAngle = this->controlJoint->GetAngle(0).Radian();
#endif
    cl = cl + this->controlJointRadToCL * controlAngle;
    /// \TODO: also change cd
  }

  //!!!Check:FIX!!!
  if (this->controlJointdA)
  {
#if GAZEBO_MAJOR_VERSION >= 9
    dA = this->controlJointdA->Position(0);
#else
    dA = this->controlJointdA->GetAngle(0).Radian();
#endif
  }
  if (this->controlJointdE)
  {
#if GAZEBO_MAJOR_VERSION >= 9
    dE = this->controlJointdE->Position(0);
#else
    dE = this->controlJointdE->GetAngle(0).Radian();
#endif
  }
  if (this->controlJointdR)
  {
#if GAZEBO_MAJOR_VERSION >= 9
    dR = this->controlJointdR->Position(0);
#else
    dR = this->controlJointdR->GetAngle(0).Radian();
#endif
  }

  if (this->controlJointdF)
  {
#if GAZEBO_MAJOR_VERSION >= 9
    dF = this->controlJointdF->Position(0);
#else
    dF = this->controlJointdF->GetAngle(0).Radian();
#endif
  }

  //Get actuator deflections direclty from topic instead of control surface angle
dA=dA_defl_;
dE=dE_defl_;
dR=dR_defl_;
dF=dF_defl_;

//Testing that the delfections between controllers and gazebo model
dT_defl_=dT_defl_/3500.00*40.00;

//Compensate for Gazebo diffrent axis direction
dA=-dA;
dE=-dE;
dR=-dR;
dF=-dF;

// if(disp_count%5==0){
//   printf("Airframe_Lift_Drag\n");
//   printf("dA_defl: %f ; dE_defl: %f ; dR_defl:%f ; dT_defl:%f ; dF_defl:%f \n",dA,dE,dR,dT_defl_,dF);
//   printf("\n");
// }

//Work out Euler angles and DCM and inverse DCM
  ignition::math::Pose3d pose1;
  pose1 = this->link->WorldPose();
  ignition::math::Quaternion quat=pose1.Rot();
  ignition::math::Vector3 eul=quat.Euler();
  double phi_in_2=eul.X();
  double theta_in_2=eul.Y();
  double psi_in_2=eul.Z();
  //DCM and InvDCM for Gazebo axes. To get plane axes DCM multply theta_in_2 and psi_in_2 by -1
  ignition::math::Matrix3 DCM=ignition::math::Matrix3(cos(psi_in_2)*cos(theta_in_2),sin(psi_in_2)*cos(theta_in_2),-sin(theta_in_2),cos(psi_in_2)*sin(theta_in_2)*sin(phi_in_2)-sin(psi_in_2)*cos(phi_in_2),sin(psi_in_2)*sin(theta_in_2)*sin(phi_in_2)+cos(psi_in_2)*cos(phi_in_2),cos(theta_in_2)*sin(phi_in_2),cos(psi_in_2)*sin(theta_in_2)*cos(phi_in_2)+sin(psi_in_2)*sin(phi_in_2),sin(psi_in_2)*sin(theta_in_2)*cos(phi_in_2)-cos(psi_in_2)*sin(phi_in_2),cos(theta_in_2)*cos(phi_in_2));
  ignition::math::Matrix3 InvDCM=ignition::math::Matrix3(cos(psi_in_2)*cos(theta_in_2),cos(psi_in_2)*sin(theta_in_2)*sin(phi_in_2)-sin(psi_in_2)*cos(phi_in_2),cos(psi_in_2)*sin(theta_in_2)*cos(phi_in_2)+sin(psi_in_2)*sin(phi_in_2),sin(psi_in_2)*cos(theta_in_2),sin(psi_in_2)*sin(theta_in_2)*sin(phi_in_2)+cos(psi_in_2)*cos(phi_in_2),sin(psi_in_2)*sin(theta_in_2)*cos(phi_in_2)-cos(psi_in_2)*sin(phi_in_2),-sin(theta_in_2),cos(theta_in_2)*sin(phi_in_2),cos(theta_in_2)*cos(phi_in_2));

  //Testing initialisation Heading issue
  // if(disp_count%5==0){
  //   printf("Airframe_Lift_Drag\n");
  //   printf("Psi_Gazebo_Ax_comp : %f \n",-psi_in_2*180/M_PI+90);
  //   printf("\n");
  // }

  //Get inertial angular velocity !!!Check!!!
  ignition::math::Vector3d wi_g=this->link->WorldAngularVel();
  double Pi=wi_g.X()-custom_wind_ang_vel_.X();
  double Qi=wi_g.Y()-custom_wind_ang_vel_.Y();
  double Ri=wi_g.Z()-custom_wind_ang_vel_.Z();
  ignition::math::Vector3d wi =ignition::math::Vector3d(Pi,Qi,Ri);

  //Get body angular velocity
  ignition::math::Vector3 wb=DCM.operator*(wi);
  double P=wb.X();
  double Q=wb.Y();
  double R=wb.Z();



  //Get body angular velocity !!!Check!!!
  // ignition::math::Vector3d wb=this->link->RelativeAngularVel();
  // double P=wb.X();
  // double Q=wb.Y();
  // double R=wb.Z();

  //Compensate for different axes in Gazebo
  Q=-Q;
  R=-R;



/*Custom Wind
Wind calculations done in Gazebo Axes
*/

double now=(this->world->SimTime()).Double();
double T =now-_last_time;//time step for wind plugin

if(T>=1/pub_rate){ //only update wind when timestep time passed

  // std::cout<<GAZEBO_MAJOR_VERSION<<"\n";

  _last_time=now;

  /*Gust*/

  // double current_sim_time=(double)sim_time_.sec()+(double)sim_time_.nsec()/1e9;
  double current_sim_time=now;
  double V_gust;
  ignition::math::Vector3d position=pose.Pos();

  if(position.Z()>1 && !flight_start){
    flight_start=true;
    flight_start_time=current_sim_time;
  }

  // printf("T :            %f \n" ,T);
  // printf("current_sim_time:%f \n",current_sim_time);

  //All axes
  track_length=track_length+sqrt(pow(position.X()-prev_position.X(),2)+pow(position.Y()-prev_position.Y(),2)+pow(position.Z()-prev_position.Z(),2));
  prev_position=position;
  if(flight_start){ //See if there is a better way to got get start time. Simulation time depends on when Gazebo is started and the time it takes to arm.

  if((current_sim_time-flight_start_time)<(this->ts)){ //first section
    V_gust=0;
  }else if(!gust_start[0]){
    d_start=track_length; //set x zero position !!!CHECK IF RIGHT!!!
    gust_start[0]=true;
  }else if(gust_start[0] && (track_length-d_start)<=this->dm){ //second section
    V_gust=(this->Vm/2*(1-cos((M_PI*(track_length-d_start))/this->dm)));
  }else if(gust_start[0] && (track_length-d_start)>this->dm && (track_length-d_start)<(this->dm+this->ds)){
    V_gust=(this->Vm);
  }else if(gust_start[0] && (track_length-d_start)>=(this->dm+this->ds) && (track_length-d_start)<=(2*this->dm+this->ds)){
    V_gust=(this->Vm/2*(1+cos((M_PI*(track_length-(d_start+this->dm+this->ds)))/this->dm)));
  }else if(gust_start[0] && (track_length-d_start)>(2*this->dm+this->ds)){
    V_gust=0;
  }

  }else{
    V_gust=0;
  }

  /*Turbulence*/
  // //Assume aircraft will be under 1000ft(304.8m) !!!Check if h and V are in matric or imperial units!!!
  // //Convert from metric to imperial units
  h=position.Z()*3.28084;
  double ground_speed_imp=vel.Length()*3.28084;
  double b_imp=this->b*3.28084;

  double y0u=0,y0v=0,y0w=0,y0p=0,y0q=0,y0r=0;
  if(h<20){ //under 20ft no turbulence
    y0u=0;y0v=0;y0w=0;y0p=0;y0q=0;y0r=0;
  }else{
  //Filter variables
  double L_w=h/2;
  double L_u=h/pow((0.177+0.000823*h),1.2);
  double L_v=L_u/2;

  double sigma_w=0.1*(V_20*3.28084);
  double sigma_u=sigma_w/pow((0.177+0.000823*h),0.4);
  double sigma_v=sigma_u;



  //White noise (!!!Try to make a better implementation of white noise!!!)
  double x0u=white_noise_distribution(white_noise);
  double x0p=white_noise_distribution(white_noise);
  double x0v=white_noise_distribution(white_noise);
  double x0r=white_noise_distribution(white_noise);
  double x0w=white_noise_distribution(white_noise);
  double x0q=white_noise_distribution(white_noise);

  //Longitudinal
  //force
  double A1u=sigma_u*sqrt((2*L_u)/(M_PI*ground_speed_imp));
  double A2u=L_u/ground_speed_imp;
  y0u=1/(T+2*A2u)*(A1u*T*x0u+A1u*T*x1u-(T-2*A2u)*y1u);
  y1u=y0u;
  x1u=x0u;
  //moment
  double A1p=sigma_w*sqrt(0.8/ground_speed_imp)*(pow(M_PI/(4*b_imp),1/6))/(pow((2*L_w),1/3));
  double A2p=(4*b_imp)/(M_PI*ground_speed_imp);
  y0p=1/(T+2*A2p)*(A1p*T*x0p+A1p*T*x1p-(T-2*A2p)*y1p);
  y1p=y0p;
  x1p=x0p;
  //Lateral
  //force
  double A1v=sigma_v*sqrt((2*L_v)/(M_PI*ground_speed_imp));
  double A2v=(2*sqrt(3)*L_v)/ground_speed_imp;
  double A3v=(2*L_v)/ground_speed_imp;
  y0v=1/(pow(T,2)+4*A3v*T+4*pow(A3v,2))*((pow(T,2)*A1v+2*T*A1v*A2v)*x0v+(2*A1v*pow(T,2))*x1v+(A1v*T-2*A1v*A2v)*x2v-(2*pow(T,2)-8*pow(A3v,2))*y1v-(pow(T,2)-4*A3v*T+4*pow(A3v,2))*y2v);
  y2v=y1v;
  y1v=y0v;
  x2v=x1v;
  x1v=x0v;
  //moment
  double A1r=A1v;
  double A2r=A2v;
  double A3r=A3v;
  double A4r=(3*b_imp)/(M_PI*ground_speed_imp);
  double A5r=1/ground_speed_imp;
  y0r=1/(pow(T,3)+2*pow(T,2)*(2*A3r+A4r)+4*T*(pow(A3r,2)+2*A3r*A4r)+8*pow(A3r,2)*A4r)*(x0r*(2*A1r*A5r*pow(T,2)+4*T*A1r*A2r*A5r)+x1r*(2*A1r*A5r*pow(T,2)-4*T*A1r*A2r*A5r)+x2r*(-2*A1r*A5r*pow(T,2)-4*T*A1r*A2r*A5r)+x3r*(-2*A1r*A5r*pow(T,2)+4*T*A1r*A2r*A5r)-y1r*(3*pow(T,3)+2*pow(T,2)*(2*A3r+A4r)-4*T*(pow(A3r,2)+2*A3r*A4r)-24*pow(A3r,2)*A4r)-y2r*(3*pow(T,3)-2*pow(T,2)*(2*A3r+A4r)-4*T*(pow(A3r,2)+2*A3r*A4r)+24*pow(A3r,2)*A4r)-y3r*(pow(T,3)-2*pow(T,2)*(2*A3r+A4r)+4*T*(pow(A3r,2)+2*A3r*A4r)-8*pow(A3r,2)*A4r));
  y3r=y2r;
  y2r=y1r;
  y1r=y0r;
  x3r=x2r;
  x2r=x1r;
  x1r=x0r;
  //Vertical
  //force
  double A1w=sigma_w*sqrt((2*L_w)/(M_PI*ground_speed_imp));
  double A2w=(2*sqrt(3)*L_w)/ground_speed_imp;
  double A3w=(2*L_w)/ground_speed_imp;
  y0w=1/(pow(T,2)+4*A3w*T+4*pow(A3w,2))*((pow(T,2)*A1w+2*T*A1w*A2w)*x0w+(2*A1w*pow(T,2))*x1w+(A1w*T-2*A1w*A2w)*x2w-(2*pow(T,2)-8*pow(A3w,2))*y1w-(pow(T,2)-4*A3w*T+4*pow(A3w,2))*y2w);
  y2w=y1w;
  y1w=y0w;
  x2w=x1w;
  x1w=x0w;
  //moment
  double A1q=A1w;
  double A2q=A2w;
  double A3q=A3w;
  double A4q=(4*b_imp)/(M_PI*ground_speed_imp);
  double A5q=1/ground_speed_imp;
  y0q=1/(pow(T,3)+2*pow(T,2)*(2*A3q+A4q)+4*T*(pow(A3q,2)+2*A3q*A4q)+8*pow(A3q,2)*A4q)*(x0q*(2*A1q*A5q*pow(T,2)+4*T*A1q*A2q*A5q)+x1q*(2*A1q*A5q*pow(T,2)-4*T*A1q*A2q*A5q)+x2q*(-2*A1q*A5q*pow(T,2)-4*T*A1q*A2q*A5q)+x3q*(-2*A1q*A5q*pow(T,2)+4*T*A1q*A2q*A5q)-y1q*(3*pow(T,3)+2*pow(T,2)*(2*A3q+A4q)-4*T*(pow(A3q,2)+2*A3q*A4q)-24*pow(A3q,2)*A4q)-y2q*(3*pow(T,3)-2*pow(T,2)*(2*A3q+A4q)-4*T*(pow(A3q,2)+2*A3q*A4q)+24*pow(A3q,2)*A4q)-y3q*(pow(T,3)-2*pow(T,2)*(2*A3q+A4q)+4*T*(pow(A3q,2)+2*A3q*A4q)-8*pow(A3q,2)*A4q));
  y3q=y2q;
  y2q=y1q;
  y1q=y0q;
  x3q=x2q;
  x2q=x1q;
  x1q=x0q;

  //Convert from imperial to metric
  y0u=y0u/3.28084;
  y0p=y0p;
  y0v=y0v/3.28084;
  y0r=y0r;
  y0w=y0w/3.28084;
  y0q=y0q;

  }

  //Convert from turbulence from wind axis to inertial axis
  ignition::math::Matrix3 DCM_BW=ignition::math::Matrix3((double)cos(this->alpha)*cos(this->beta),(double)sin(this->beta),(double)sin(this->alpha)*cos(this->beta),(double)-cos(this->alpha)*sin(this->beta),(double)cos(this->beta),(double)-sin(this->alpha)*sin(this->beta),(double)-sin(this->alpha),(double)0,(double)cos(this->alpha));
  ignition::math::Matrix3 DCM_WB=DCM_BW.Inverse();
  ignition::math::Matrix3 DCM_WI=InvDCM.operator*(DCM_WB);
  ignition::math::Vector3d Turb_force_inertial=DCM_WI.operator*(ignition::math::Vector3d(y0u,y0v,y0w));
  ignition::math::Vector3d Turb_moment_inertial=DCM_WI.operator*(ignition::math::Vector3d(y0p,y0q,y0r));

  /*Shear*/
  double V_shear=0;
  double h_shear=pose.Z()*3.28084;
  // double z0=2;//0.15 for takeoff, approach and landing and 2 for other phases
  double z0=0.15;
  if(h_shear>3 && h_shear<1000){
    V_shear=this->V_20_shear*(log(h_shear/z0))/(log(20/z0));
  }
  //Convert to metric
  V_shear=V_shear/3.28084;


  /*Net effect of wind*/
  //All Wind
  double lin_wind_x=V_gust*(double)((this->gust_direction).X())+Turb_force_inertial.X()+V_shear*(double)((this->shear_direction).X());
  double lin_wind_y=V_gust*(double)((this->gust_direction).Y())+Turb_force_inertial.Y()+V_shear*(double)((this->shear_direction).Y());
  double lin_wind_z=V_gust*(double)((this->gust_direction).Z())+Turb_force_inertial.Z()+V_shear*(double)((this->shear_direction).Z());
  custom_wind_lin_vel_=ignition::math::Vector3d(lin_wind_x,lin_wind_y,lin_wind_z);
  custom_wind_ang_vel_=Turb_moment_inertial;
  // printf("lin_wind_x : %f ; lin_wind_y : %f ; lin_wind_z : %f ; \n",lin_wind_x,lin_wind_y,lin_wind_z);
  // printf("ang_wind_x : %f ; ang_wind_y : %f ; ang_wind_z : %f ; \n",y0p,y0q,y0r);

  //Testing
  custom_wind_lin_vel_ = ignition::math::Vector3d(0,0,0);
  custom_wind_ang_vel_ = ignition::math::Vector3d(0,0,0);


  //Publish wind speed
  gazebo::msgs::Vector3d* wind_v = new gazebo::msgs::Vector3d();
  wind_v->set_x(custom_wind_lin_vel_.X());
  wind_v->set_y(custom_wind_lin_vel_.Y());
  wind_v->set_z(custom_wind_lin_vel_.Z());

  wind_msg.set_frame_id(frame_id_);
  wind_msg.set_time_usec(now * 1e6);
  wind_msg.set_allocated_velocity(wind_v);

  wind_pub_->Publish(wind_msg);
}


  // compute lift force at cp
  ignition::math::Vector3d lift = cl * q * this->area * liftI;

  // compute cd at cp, check for stall, correct for sweep
  double cd;
  if (this->alpha > this->alphaStall)
  {
    cd = (this->cda * this->alphaStall +
          this->cdaStall * (this->alpha - this->alphaStall))
         * cosSweepAngle;
  }
  else if (this->alpha < -this->alphaStall)
  {
    cd = (-this->cda * this->alphaStall +
          this->cdaStall * (this->alpha + this->alphaStall))
         * cosSweepAngle;
  }
  else
    cd = (this->cda * this->alpha) * cosSweepAngle;

  // make sure drag is positive
  cd = fabs(cd);

  // drag at cp
  ignition::math::Vector3d drag = cd * q * this->area * dragDirection;

  // compute cm at cp, check for stall, correct for sweep
  double cm;
  if (this->alpha > this->alphaStall)
  {
    cm = (this->cma * this->alphaStall +
          this->cmaStall * (this->alpha - this->alphaStall))
         * cosSweepAngle;
    // make sure cm is still great than 0
    cm = std::max(0.0, cm);
  }
  else if (this->alpha < -this->alphaStall)
  {
    cm = (-this->cma * this->alphaStall +
          this->cmaStall * (this->alpha + this->alphaStall))
         * cosSweepAngle;
    // make sure cm is still less than 0
    cm = std::min(0.0, cm);
  }
  else
    cm = this->cma * this->alpha * cosSweepAngle;

  // Take into account the effect of control surface deflection angle to Cm
  cm += this->cm_delta * controlAngle;

  //!!New coefficients for single airframe!! Check if vbar is speedInLDPlane or magnitude of vel
  double C_L;
  double C_D;
  double C_y;
  double C_l;
  double C_n;
  double C_m;
  double C_X;
  double C_Z;
  double C_l_B;
  double C_n_B;
  double Ps;
  double Rs;

  Ps=P*cos(this->alpha)+R*sin(this->alpha);
  Rs=-P*sin(this->alpha)+R*cos(this->alpha);
  //Testing !!!REMOVE WHEN DONE!!!
  // Ps=P;
  // Rs=R;
  //this->beta=0;

  C_L = this->C_L_0 + this->C_L_alpha*this->alpha+cbar/(2*total_speed)*this->C_L_Q*Q+this->C_L_dE*dE+this->C_L_dF*dF;//figure out how to get wb(P,Q,R)
  C_D=this->C_D_0+((C_L)*(C_L))/(M_PI*this->A*this->e);
  C_y=this->C_y_beta*this->beta+this->b/(2*total_speed)*this->C_y_P*Ps+this->b/(2*total_speed)*this->C_y_R*Rs+this->C_y_dA*dA+this->C_y_dR*dR;//figure out how to get wb(P,Q,R) and beta
  C_l=this->C_l_beta*this->beta+this->b/(2*total_speed)*this->C_l_P*Ps+this->b/(2*total_speed)*this->C_l_R*Rs+this->C_l_dA*dA+this->C_l_dR*dR;
  C_n=this->C_n_beta*this->beta+this->b/(2*total_speed)*this->C_n_P*Ps+this->b/(2*total_speed)*this->C_n_R*Rs+this->C_n_dA*dA+this->C_n_dR*dR;
  C_m=this->C_m_0+this->C_m_alpha*this->alpha+this->cbar/(2*total_speed)*this->C_m_Q*Q+this->C_m_dE*dE+this->C_m_dF*dF;
  C_X=-C_D*cos(this->alpha)+C_L*sin(this->alpha);
  C_Z=-C_L*cos(this->alpha)-C_D*sin(this->alpha);
  C_l_B=C_l*cos(this->alpha)-C_n*sin(this->alpha);
  C_n_B=C_n*cos(this->alpha)+C_l*sin(this->alpha);

  //Testing !!!REMOVE WHEN DONE!!!
  // C_l_B=C_l;
  // C_n_B=C_n;
//   if(disp_count%5==0){
//   printf("C_y: %f ; C_l: %f ; C_n:%f ; \n",C_y,C_l,C_n);
//   printf("\n");
// }

  // Workout forces at centre of mass in body frame
  double X=q*this->S*C_X;
  double Y=q*this->S*C_y;
  double Z=q*this->S*C_Z;

  //compensate for Gazebo axes direction definition
  //Y=-Y;Z=-Z;
  Y=-Y;Z=-Z;
  // Convert body force to inertial frame
  ignition::math::Vector3 force=ignition::math::Vector3(X,Y,Z);
  //ignition::math::Vector3 force=InvDCM.operator*(forceb);

  // Workout moments at centre of mass in body frame
  double L=q*this->S*this->b*C_l_B;
  double M=q*this->S*this->cbar*C_m;
  double N=q*this->S*this->b*C_n_B;
  //compensate for Gazebo axes direction definition
  //M=-M;N=-N;
  M=-M;N=-N;
  ignition::math::Vector3 moment=ignition::math::Vector3(L,M,N);
  //ignition::math::Vector3 moment=InvDCM.operator*(momentb);

  // compute moment (torque) at cp
  // ignition::math::Vector3d moment = cm * q * this->area * momentDirection;

// #if GAZEBO_MAJOR_VERSION >= 9
//   ignition::math::Vector3d cog = this->link->GetInertial()->CoG();
// #else
//   ignition::math::Vector3d cog = ignitionFromGazeboMath(this->link->GetInertial()->GetCoG());
// #endif

//   // force about cg in inertial frame
//   ignition::math::Vector3d force = lift + drag;

//   force=ignition::math::Vector3d(0.0,0.0,0.0);
//   moment=ignition::math::Vector3d(0.0,0.0,0.0);


  // debug
  //
  // if ((this->link->GetName() == "wing_1" ||
  //      this->link->GetName() == "wing_2") &&
  //     (vel.Length() > 50.0 &&
  //      vel.Length() < 50.0))
  if (0)
  {
    gzdbg << "=============================\n";
    gzdbg << "sensor: [" << this->GetHandle() << "]\n";
    gzdbg << "Link: [" << this->link->GetName()
          << "] pose: [" << pose
          << "] dynamic pressure: [" << q << "]\n";
    gzdbg << "spd: [" << vel.Length()
          << "] vel: [" << vel << "]\n";
    gzdbg << "LD plane spd: [" << velInLDPlane.Length()
          << "] vel : [" << velInLDPlane << "]\n";
    gzdbg << "forward (inertial): " << forwardI << "\n";
    gzdbg << "upward (inertial): " << upwardI << "\n";
    gzdbg << "lift dir (inertial): " << liftI << "\n";
    gzdbg << "Span direction (normal to LD plane): " << spanwiseI << "\n";
    gzdbg << "sweep: " << this->sweep << "\n";
    gzdbg << "alpha: " << this->alpha << "\n";
    gzdbg << "lift: " << lift << "\n";
    gzdbg << "drag: " << drag << " cd: "
          << cd << " cda: " << this->cda << "\n";
    gzdbg << "moment: " << moment << "\n";
    gzdbg << "force: " << force << "\n";
    gzdbg << "moment: " << moment << "\n";
  }

  // Correct for nan or inf
  force.Correct();
  this->cp.Correct();
  moment.Correct();

  // apply forces at cg (with torques for position shift)
  // this->link->AddForceAtRelativePosition(force, this->cp);
  // this->link->AddTorque(moment);

  //add force and torque on the aircraft
  this->link->AddRelativeForce(force);
  this->link->AddRelativeTorque(moment);

}

// void AirframeActualLiftDragPlugin::WindVelocityCallback(const boost::shared_ptr<const physics_msgs::msgs::Wind> &msg) {
//   wind_vel_ = ignition::math::Vector3d(msg->velocity().x(),
//             msg->velocity().y(),
//             msg->velocity().z());
// }

void AirframeActualLiftDragPlugin::ActuatorDeflectionCallback(ActuatorDeflectionsPtr &deflections) {
  dA_defl_=deflections->da();
  dE_defl_=deflections->de();
  dF_defl_=deflections->df();
  dR_defl_=deflections->dr();
  dT_defl_=deflections->dt();
}

void AirframeActualLiftDragPlugin::WorldStatsCallback(ConstWorldStatisticsPtr &_msg){
  sim_time_=_msg->sim_time();

}
