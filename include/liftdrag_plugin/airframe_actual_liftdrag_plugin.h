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
#ifndef _GAZEBO_LIFT_DRAG_PLUGIN_HH_
#define _GAZEBO_LIFT_DRAG_PLUGIN_HH_

#include <string>
#include <vector>
#include <boost/bind.hpp>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include <ignition/math.hh>
#include <gazebo/gazebo.hh>
#include "ActuatorDeflections.pb.h"
#include <math.h>
#include <random>
#include <iostream>
#include <iomanip>
#include <string>
#include <map>
#include <cmath>
#include "StateMachineState.pb.h"

#include "Wind.pb.h"

namespace gazebo
{
  typedef const boost::shared_ptr<const act_msgs::msgs::ActuatorDeflections> ActuatorDeflectionsPtr;
  typedef const boost::shared_ptr<const mp_msgs::msgs::StateMachineState> StateMachineStatePtr;
  /// \brief A plugin that simulates lift and drag.
  class GAZEBO_VISIBLE AirframeActualLiftDragPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: AirframeActualLiftDragPlugin();

    /// \brief Destructor.
    public: ~AirframeActualLiftDragPlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Callback for World Update events.
    protected: virtual void OnUpdate();

    /// \brief Connection to World Update events.
    protected: event::ConnectionPtr updateConnection;

    /// \brief Pointer to world.
    protected: physics::WorldPtr world;

    /// \brief Pointer to physics engine.
    protected: physics::PhysicsEnginePtr physics;

    /// \brief Pointer to model containing plugin.
    protected: physics::ModelPtr model;

    /// \brief Coefficient of Lift / alpha slope.
    /// Lift = C_L * q * S
    /// where q (dynamic pressure) = 0.5 * rho * v^2
    protected: double cla;

    /// \brief Coefficient of Drag / alpha slope.
    /// Drag = C_D * q * S
    /// where q (dynamic pressure) = 0.5 * rho * v^2
    protected: double cda;

    /// \brief Coefficient of Moment / alpha slope.
    /// Moment = C_M * q * S
    /// where q (dynamic pressure) = 0.5 * rho * v^2
    protected: double cma;

    /// \brief angle of attack when airfoil stalls
    protected: double alphaStall;

    /// \brief Cl-alpha rate after stall
    protected: double claStall;

    /// \brief Cd-alpha rate after stall
    protected: double cdaStall;

    /// \brief Cm-alpha rate after stall
    protected: double cmaStall;

    /// \breif Coefficient of Moment / control surface deflection angle slope
    protected: double cm_delta;

    /// \brief: \TODO: make a stall velocity curve
    protected: double velocityStall;

    /// \brief air density
    /// at 25 deg C it's about 1.1839 kg/m^3
    /// At 20 °C and 101.325 kPa, dry air has a density of 1.2041 kg/m3.
    protected: double rho;

    /// \brief if the shape is aerodynamically radially symmetric about
    /// the forward direction. Defaults to false for wing shapes.
    /// If set to true, the upward direction is determined by the
    /// angle of attack.
    protected: bool radialSymmetry;

    /// \brief effective planeform surface area
    protected: double area;

    /// \brief angle of sweep
    protected: double sweep;

    /// \brief initial angle of attack
    protected: double alpha0;

    /// \brief angle of attack
    protected: double alpha;

    //single airframe constants
    protected: double S;
    protected: double b;
    protected: double cbar;
    protected: double A;
    protected: double e;
    protected: double beta; //slip angle

    /// \brief center of pressure in link local coordinates
    protected: ignition::math::Vector3d cp;

    /// \brief Normally, this is taken as a direction parallel to the chord
    /// of the airfoil in zero angle of attack forward flight.
    protected: ignition::math::Vector3d forward;

    /// \brief A vector in the lift/drag plane, perpendicular to the forward
    /// vector. Inflow velocity orthogonal to forward and upward vectors
    /// is considered flow in the wing sweep direction.
    protected: ignition::math::Vector3d upward;

    /// \brief Pointer to link currently targeted by mud joint.
    protected: physics::LinkPtr link;

    /// \brief Pointer to a joint that actuates a control surface for
    /// this lifting body
    protected: physics::JointPtr controlJoint;

    //Pointers of aileron, elevator and rudder
    protected: physics::JointPtr controlJointdA;
    protected: physics::JointPtr controlJointdE;
    protected: physics::JointPtr controlJointdR;
    protected: physics::JointPtr controlJointdF;

    /// \brief how much to change CL per radian of control surface joint
    /// value.
    protected: double controlJointRadToCL;

    //single airframe aerodynamic coeffecinets
    protected: double C_L_0;
    protected: double C_L_alpha;
    protected: double C_L_Q;
    protected: double C_L_dE;
    protected: double C_L_dF;
    protected: double C_D_0;
    protected: double C_y_beta;
    protected: double C_y_P;
    protected: double C_y_R;
    protected: double C_y_dA;
    protected: double C_y_dR;
    protected: double C_l_beta;
    protected: double C_l_P;
    protected: double C_l_R;
    protected: double C_l_dA;
    protected: double C_l_dR;
    protected: double C_n_beta;
    protected: double C_n_P;
    protected: double C_n_R;
    protected: double C_n_dA;
    protected: double C_n_dR;
    protected: double C_m_0;
    protected: double C_m_alpha;
    protected: double C_m_Q;
    protected: double C_m_dE;
    protected: double C_m_dF;

    protected: int disp_count=0;

    protected: double h;
    protected: double V_20;//wind velocity at 20ft
    protected: double _last_time=0;
    protected:
      // ignition::math::Vector3d Vm;
      // ignition::math::Vector3d ts;
      // ignition::math::Vector3d dm;
      // ignition::math::Vector3d ds;
      double Vm;
      double ts;
      double dm;
      double ds;
      gazebo::msgs::Time sim_time_;
      ignition::math::Vector3d gust_direction;
      double track_length=0;
      ignition::math::Vector3d prev_position;
      double V_20_shear;
      ignition::math::Vector3d shear_direction;

    private:
      std::random_device rd;
      std::default_random_engine white_noise{rd()};
      std::uniform_real_distribution<double> white_noise_distribution{-1.0, 1.0};// !!!check if range is fine!!!

      // std::default_random_engine white_noise;
      // std::normal_distribution<double> white_noise_distribution;

      double x1u=0;
      double y1u=0;
      double x1p=0;
      double y1p=0;
      double x1v=0;
      double x2v=0;
      double y1v=0;
      double y2v=0;
      double x1w=0;
      double x2w=0;
      double y1w=0;
      double y2w=0;
      double x1r=0;
      double x2r=0;
      double x3r=0;
      double y1r=0;
      double y2r=0;
      double y3r=0;
      double x1q=0;
      double x2q=0;
      double x3q=0;
      double y1q=0;
      double y2q=0;
      double y3q=0;

      ignition::math::Vector3d custom_wind_lin_vel_ = ignition::math::Vector3d(0,0,0);
      ignition::math::Vector3d custom_wind_ang_vel_ = ignition::math::Vector3d(0,0,0);
      bool gust_start[3]={false,false,false};
      bool flight_start=false;
      double flight_start_time=0;
      // ignition::math::Vector3d d_start;
      double d_start;

    /// \brief SDF for this plugin;
    protected: sdf::ElementPtr sdf;

    //private: void WindVelocityCallback(const boost::shared_ptr<const physics_msgs::msgs::Wind> &msg);

    private: transport::NodePtr node_handle_;
    private: transport::SubscriberPtr wind_sub_;
    private: std::string namespace_;
    private: std::string wind_sub_topic_ = "world_wind";
    // private: ignition::math::Vector3d wind_vel_;
    private: std::string act_def_sub_topic_;
    private: transport::SubscriberPtr world_stats_sub_;
    private: std::string world_stats_sub_topic_ = "world_stats";

    private: void WorldStatsCallback(ConstWorldStatisticsPtr &_msg);

    private:
      double dA_defl_;
      double dE_defl_;
      double dF_defl_;
      double dR_defl_;
      double dT_defl_;
      transport::SubscriberPtr act_def_sub_;
      void ActuatorDeflectionCallback(ActuatorDeflectionsPtr &deflections);

    private:
      int sm_state_out=0;
      std::string sm_state_sub_topic_;
      transport::SubscriberPtr sm_state_sub_;
      void StateMachineStateCallback(StateMachineStatePtr &sm_msg);
      bool disp_fw_gazebo_loc=false;

    private: ignition::math::Vector3d wind_vel_=ignition::math::Vector3d(0,0,0);
    private: double now=0;
    private:
      //Wind velocity publishing
      std::string frame_id_="base_link";
      std::string wind_pub_topic_ = "world_wind";
      transport::PublisherPtr wind_pub_;
      physics_msgs::msgs::Wind wind_msg;
      double pub_rate = 2.0;

  };
}
#endif
