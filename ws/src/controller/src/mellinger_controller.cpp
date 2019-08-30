#include <ros/ros.h>
#include "mellinger_controller.h"
#include <math.h>
#include "math3d.h"
#include <stdio.h>
#include <Eigen/Geometry>

//EXTRA INCLUDES
#include <ctime>
#include "mpc_wrapper.h"


#define GRAVITY_MAGNITUDE (9.81f)
#define MIN_THRUST 2.0
#define MAX_THRUST 20.0
#define MAX_BODYRATE_Z 2.0
#define MAX_BODYRATE_XY 3.0
#define MIN_BODYRATE_Z -MAX_BODYRATE_Z
#define MIN_BODYRATE_XY -MAX_BODYRATE_XY


//STATE
#define kPosX 0
#define kPosY 1
#define kPosZ 2
#define kOriW 3
#define kOriX 4
#define kOriY 5
#define kOriZ 6
#define kVelX 7
#define kVelY 8
#define kVelZ 9

//INPUT
#define kThrust 0
#define kRateX 1
#define kRateY 2
#define kRateZ 3


#define do_preparation_step false

//ACADO/MPC
using namespace rpg_mpc;

 MpcWrapper<float> mpc_wrapper_ = MpcWrapper<float>();
Eigen::Matrix<float, kStateSize, 1> est_state_;

Eigen::Matrix<float, kStateSize, kSamples+1> reference_states_ = Eigen::Matrix<float, kStateSize, kSamples+1>::Zero();
Eigen::Matrix<float, kInputSize, kSamples+1> reference_inputs_ = Eigen::Matrix<float, kInputSize, kSamples+1>::Zero();
Eigen::Matrix<float, kStateSize, kSamples+1> predicted_states_ = Eigen::Matrix<float, kStateSize, kSamples+1>::Zero();
Eigen::Matrix<float, kInputSize, kSamples> predicted_inputs_ = Eigen::Matrix<float, kInputSize, kSamples>::Zero();
  Eigen::Quaternionf q;

namespace controller {

  // Initialize.
  bool MellingerController::Initialize(const ros::NodeHandle& n)
 {
    name_ = ros::names::append(n.getNamespace(), "controller");

	est_state_ <<  0, 0, 0, 1, 0, 0, 0, 0, 0, 0; 
	 
    if (!LoadParameters(n)) {
      ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
      return false;
    }

    if (!RegisterCallbacks(n)) {
      ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
      return false;
    }

    // if (!LoadFromDisk()) {
    //   ROS_ERROR("%s: Failed to load K, x_ref, u_ref from disk.", name_.c_str());
    //   return false;
    // }

    // Set up control publisher.
    ros::NodeHandle nl(n);
    control_pub_ = nl.advertise<testbed_msgs::ControlStamped>(
        control_topic_.c_str(), 1, false);

    Reset();

    initialized_ = true;
    return true;
  }

  // Load parameters. This may be overridden by derived classes.
  bool MellingerController::LoadParameters(const ros::NodeHandle& n) {
    ros::NodeHandle nl(n);

    // Controller Parameters
    if (!nl.getParam("g_vehicleMass", g_vehicleMass)) return false;
    if (!nl.getParam("massThrust", massThrust)) return false;

    if (!nl.getParam("kp_xy", kp_xy)) return false;
    if (!nl.getParam("kd_xy", kd_xy)) return false;
    if (!nl.getParam("ki_xy", ki_xy)) return false;
    if (!nl.getParam("i_range_xy", i_range_xy)) return false;

    if (!nl.getParam("kp_z", kp_z)) return false;
    if (!nl.getParam("kd_z", kd_z)) return false;
    if (!nl.getParam("ki_z", ki_z)) return false;
    if (!nl.getParam("i_range_z", i_range_z)) return false;

    if (!nl.getParam("kR_xy", kR_xy)) return false;
    if (!nl.getParam("kw_xy", kw_xy)) return false;
    if (!nl.getParam("ki_m_xy", ki_m_xy)) return false;
    if (!nl.getParam("i_range_m_xy", i_range_m_xy)) return false;

    if (!nl.getParam("kR_z", kR_z)) return false;
    if (!nl.getParam("kw_z", kw_z)) return false;
    if (!nl.getParam("ki_m_z", ki_m_z)) return false;
    if (!nl.getParam("i_range_m_z", i_range_m_z)) return false;

    if (!nl.getParam("kd_omega_rp", kd_omega_rp)) return false;

    if (!nl.getParam("kpq_rates", kpq_rates_)) return false;
    if (!nl.getParam("kr_rates", kr_rates_)) return false;

    if (!nl.getParam("i_error_x", i_error_x)) return false;
    if (!nl.getParam("i_error_y", i_error_y)) return false;
    if (!nl.getParam("i_error_z", i_error_z)) return false;

    if (!nl.getParam("i_error_m_x", i_error_m_x)) return false;
    if (!nl.getParam("i_error_m_y", i_error_m_y)) return false;
    if (!nl.getParam("i_error_m_z", i_error_m_z)) return false;

    // Topics.
    if (!nl.getParam("topics/state", state_topic_)) return false;
    if (!nl.getParam("topics/setpoint", setpoint_topic_)) return false;
    if (!nl.getParam("topics/control", control_topic_)) return false;

    // Control Mode
    if (!nl.getParam("control_mode", (int&)ctrl_mode_)) return false;
    
    return true;
  }

  // Register callbacks.
  bool MellingerController::RegisterCallbacks(const ros::NodeHandle& n) {
    ros::NodeHandle nl(n);

    // Subscribers.
    state_sub_ = nl.subscribe(
        state_topic_.c_str(), 1, &MellingerController::StateCallback, this);

    setpoint_sub_ = nl.subscribe(
        setpoint_topic_.c_str(), 1, &MellingerController::SetpointCallback, this);

    return true;
  }

  // Reset variables
  void MellingerController::Reset(void)
  {
    received_setpoint_ = false;
  reference_states_.setZero();
  reference_inputs_.setZero();
  q.w();
  q.x();
  q.y();
  q.z();
  }

  // Process an incoming setpoint point change.
  void MellingerController::SetpointCallback(
      const testbed_msgs::ControlSetpoint::ConstPtr& msg) {
   
    received_setpoint_ = true;

//set ref 
  reference_states_.setZero();
  reference_inputs_.setZero();
//convert rpy to quaternion

  q = Eigen::AngleAxisf(msg->rpy.x, Eigen::Vector3f::UnitX())
	* Eigen::AngleAxisf(msg->rpy.y, Eigen::Vector3f::UnitY())
	* Eigen::AngleAxisf(msg->rpy.z, Eigen::Vector3f::UnitZ()); 
//create ref state/input
      reference_states_ = (Eigen::Matrix<float, kStateSize, 1>() <<
      msg->p.x,
      msg->p.y,
      msg->p.z,
      q.w(), 
      q.x(),
      q.y(),
      q.z(),
      msg->v.x,
      msg->v.y,
      msg->v.z
      ).finished().replicate(1, kSamples+1);

    reference_inputs_ = (Eigen::Matrix<float, kInputSize, 1>() <<
      msg->a.x,
      msg->a.y,
      msg->a.z - GRAVITY_MAGNITUDE,
      msg->brates.x,
      msg->brates.y,
      msg->brates.z
      ).finished().replicate(1, kSamples+1);
  }

  // Process an incoming state measurement.
  void MellingerController::StateCallback(
      const testbed_msgs::CustOdometryStamped::ConstPtr& msg) {
    // Catch no setpoint.
    if (!received_setpoint_)
      return;

    if (last_state_time_ < 0.0)
      last_state_time_ = ros::Time::now().toSec();

    // Read the message into the state
/*
    pos_(0) = msg->p.x;
    pos_(1) = msg->p.y;
    pos_(2) = msg->p.z;

    vel_(0) = msg->v.x;
    vel_(1) = msg->v.y;
    vel_(2) = msg->v.z;

    quat_.vec() = Vector3d (msg->q.x, msg->q.y, msg->q.z);
    quat_.w() = msg->q.w;
    quat_.normalize();
*/
  est_state_(kPosX) =  msg->p.x;
  est_state_(kPosY) =  msg->p.y;
  est_state_(kPosZ) =  msg->p.z;

  est_state_(kVelX) = msg->v.x;
  est_state_(kVelY) = msg->v.y;
  est_state_(kVelZ) = msg->v.z;

  est_state_(kOriW) = msg->q.w;
  est_state_(kOriX) = msg->q.x;
  est_state_(kOriY) = msg->q.y;
  est_state_(kOriZ) = msg->q.z;

      // Get the feedback from MPC. 
  mpc_wrapper_.setTrajectory(reference_states_, reference_inputs_); 
  mpc_wrapper_.update(est_state_, do_preparation_step); 
  mpc_wrapper_.getStates(predicted_states_);
  mpc_wrapper_.getInputs(predicted_inputs_);

 Eigen::Ref<const Eigen::Matrix<float, kStateSize, 1>> state = predicted_states_.col(0);
 Eigen::Ref<const Eigen::Matrix<float, kInputSize, 1>> input = predicted_inputs_.col(0);
 Eigen::Matrix<float, kInputSize, 1> input_bounded = input; //.template cast<float>();
  
  // Bound inputs for sanity. 
  if (input_bounded(kThrust) < MIN_THRUST){ input_bounded(kThrust) = MIN_THRUST ; }
  if (input_bounded(kThrust) > MAX_THRUST){ input_bounded(kThrust) = MAX_THRUST ; }
  if (input_bounded(kRateX) < MIN_BODYRATE_XY){ input_bounded(kRateX) = MIN_BODYRATE_XY ; }
  if (input_bounded(kRateX) > MAX_BODYRATE_XY){ input_bounded(kRateX) = MAX_BODYRATE_XY ; }
  if (input_bounded(kRateY) < MIN_BODYRATE_XY){ input_bounded(kRateY) = MIN_BODYRATE_XY ; }
  if (input_bounded(kRateY) > MAX_BODYRATE_XY){ input_bounded(kRateY) = MAX_BODYRATE_XY ; }
  if (input_bounded(kRateZ) < MIN_BODYRATE_Z){ input_bounded(kRateZ) = MIN_BODYRATE_Z ; }
  if (input_bounded(kRateZ) > MAX_BODYRATE_Z){ input_bounded(kRateZ) = MAX_BODYRATE_Z ; }

/*
all outputs of ACADO, not using orientation
  command.collective_thrust = input_bounded(INPUT::kThrust);
  command.bodyrates.x() = input_bounded(INPUT::kRateX);
  command.bodyrates.y() = input_bounded(INPUT::kRateY);
  command.bodyrates.z() = input_bounded(INPUT::kRateZ);
  command.orientation.w() = state(STATE::kOriW);
  command.orientation.x() = state(STATE::kOriX);
  command.orientation.y() = state(STATE::kOriY);
  command.orientation.z() = state(STATE::kOriZ);
*/

//Publish output of ACADO/MPC  TODO check this
    testbed_msgs::ControlStamped control_msg;
	control_msg.header.stamp = ros::Time::now();
	control_msg.control.thrust = input_bounded(0);
	control_msg.control.roll = input_bounded(1);
	control_msg.control.pitch = input_bounded(2);
	control_msg.control.yaw_dot = input_bounded(3);
	
	control_pub_.publish(control_msg);

  }

}
