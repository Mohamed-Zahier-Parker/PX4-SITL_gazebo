
#ifndef _MOVING_PLATFORM_PLUGIN_HH_
#define _MOVING_PLATFORM_PLUGIN_HH_

#include <stdio.h>

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <rotors_model/motor_model.hpp>
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "Float.pb.h"
#include "ActuatorDeflections.pb.h"

#include "common.h"

namespace gazebo {

typedef const boost::shared_ptr<const act_msgs::msgs::ActuatorDeflections> ActuatorDeflectionsPtr;

class GAZEBO_VISIBLE MovingPlatformPlugin : public ModelPlugin
{

/// \brief Constructor.
    public: MovingPlatformPlugin();

    /// \brief Destructor.
    public: ~MovingPlatformPlugin();

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

    /// \brief SDF for this plugin;
    protected: sdf::ElementPtr sdf;

    protected: physics::LinkPtr link;

    private: transport::NodePtr node_handle_;
    private: std::string namespace_;

    private: std::string uav_model_name;
    private: std::string act_def_sub_topic_;

    private:
      double dA_defl_=0;
      double dE_defl_=0;
      double dF_defl_=0;
      double dR_defl_=0;
      double dT_defl_=0;
      transport::SubscriberPtr act_def_sub_;
      void ActuatorDeflectionCallback(ActuatorDeflectionsPtr &deflections);


};
}
#endif
