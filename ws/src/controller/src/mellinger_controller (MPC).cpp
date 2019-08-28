#include <ros/ros.h>
#include "mellinger_controller.h"
#include <math.h>
#include "math3d.h"
#include <stdio.h>
#include <Eigen/Geometry>
#include "rpg_mpc/mpc_controller.h"

#include <ctime>


#define GRAVITY_MAGNITUDE (9.81f)

namespace controller {

  // Initialize.
  bool MellingerController::Initialize(const ros::NodeHandle& n) :
	  mpc_wrapper_(MpcWrapper<T>()),
	  est_state_((Eigen::Matrix<T, kStateSize, 1>() <<
	    0, 0, 0, 1, 0, 0, 0, 0, 0, 0).finished()),
	  reference_states_(Eigen::Matrix<T, kStateSize, kSamples+1>::Zero()),
	  reference_inputs_(Eigen::Matrix<T, kInputSize, kSamples+1>::Zero()),
	  predicted_states_(Eigen::Matrix<T, kStateSize, kSamples+1>::Zero()),
	  predicted_inputs_(Eigen::Matrix<T, kInputSize, kSamples>::Zero())
 {
    name_ = ros::names::append(n.getNamespace(), "controller");
    static const bool do_preparation_step = false;
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
    sp_pos_ = Vector3d::Zero();
    sp_vel_ = Vector3d::Zero();
    sp_acc_ = Vector3d::Zero();
    sp_r_pos_ = Vector3d::Zero();
    sp_r_vel_ = Vector3d::Zero();
    sp_r_acc_ = Vector3d::Zero();
    sp_brates_ = Vector3d::Zero(); 

    pos_ = Vector3d::Zero();
    vel_ = Vector3d::Zero();
    r_pos_ = Vector3d::Zero();
    r_vel_ = Vector3d::Zero();

    quat_.vec() = Vector3d::Zero();
    quat_.w() = 0;

    i_error_x = 0;
    i_error_y = 0;
    i_error_z = 0;
    i_error_m_x = 0;
    i_error_m_y = 0;
    i_error_m_z = 0;

    received_setpoint_ = false;
  }

  // Process an incoming setpoint point change.
  void MellingerController::SetpointCallback(
      const testbed_msgs::ControlSetpoint::ConstPtr& msg) {
    sp_pos_(0) = msg->p.x;
    sp_pos_(1) = msg->p.y;
    sp_pos_(2) = msg->p.z;

    sp_vel_(0) = msg->v.x;
    sp_vel_(1) = msg->v.y;
    sp_vel_(2) = msg->v.z;

    sp_acc_(0) = msg->a.x;
    sp_acc_(1) = msg->a.y;
    sp_acc_(2) = msg->a.z;

    sp_roll_ = msg->rpy.x;
    sp_pitch_ = msg->rpy.y;
    sp_yaw_ = msg->rpy.z;

    sp_brates_(0) = msg->brates.x;
    sp_brates_(1) = msg->brates.y;
    sp_brates_(2) = msg->brates.z;

    received_setpoint_ = true;

//set ref 
  reference_states_.setZero();
  reference_inputs_.setZero();

      reference_states_ = (Eigen::Matrix<T, kStateSize, 1>() <<
      msg->p.x,
      msg->p.y,
      msg->p.z,
      q_orientation.w(), // ??
      q_orientation.x(),
      q_orientation.y(),
      q_orientation.z(),
      msg->v.x,
      msg->v.y,
      msg->v.z,
      ).finished().replicate(1, kSamples+1);

    reference_inputs_ = (Eigen::Matrix<T, kInputSize, 1>() <<
      msg->a.x,
      msg->a.y,
      msg->a.z - GRAVITY_MAGNITUDE,
      msg->brates.x,
      msg->brates.y,
      msg->brates.z,
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
  est_state_(kPosX) = state_estimate.position.x();
  est_state_(kPosY) = state_estimate.position.y();
  est_state_(kPosZ) = state_estimate.position.z();
  est_state_(kOriW) = state_estimate.orientation.w();
  est_state_(kOriX) = state_estimate.orientation.x();
  est_state_(kOriY) = state_estimate.orientation.y();
  est_state_(kOriZ) = state_estimate.orientation.z();
  est_state_(kVelX) = state_estimate.velocity.x();
  est_state_(kVelY) = state_estimate.velocity.y();
  est_state_(kVelZ) = state_estimate.velocity.z();

      // Get the feedback from MPC. 
  mpc_wrapper_.setTrajectory(reference_states_, reference_inputs_); //What are ref states/inputs ??
  mpc_wrapper_.update(est_state_, do_preparation_step); 
  mpc_wrapper_.getStates(predicted_states_);
  mpc_wrapper_.getInputs(predicted_inputs_);

 Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state = predicted_states_.col(0)
 Eigen::Ref<const Eigen::Matrix<T, kInputSize, 1>> input = predicted_inputs_.col(0)
 Eigen::Matrix<T, kInputSize, 1> input_bounded = input.template cast<T>();
  
  // Bound inputs for sanity.
  input_bounded(INPUT::kThrust) = std::max(params_.min_thrust_,
    std::min(params_.max_thrust_, input_bounded(INPUT::kThrust)));
  input_bounded(INPUT::kRateX) = std::max(-params_.max_bodyrate_xy_,
    std::min(params_.max_bodyrate_xy_, input_bounded(INPUT::kRateX)));
  input_bounded(INPUT::kRateY) = std::max(-params_.max_bodyrate_xy_,
    std::min(params_.max_bodyrate_xy_, input_bounded(INPUT::kRateY)));
  input_bounded(INPUT::kRateZ) = std::max(-params_.max_bodyrate_z_,
    std::min(params_.max_bodyrate_z_, input_bounded(INPUT::kRateZ)));
/*
  command.collective_thrust = input_bounded(INPUT::kThrust);
  command.bodyrates.x() = input_bounded(INPUT::kRateX);
  command.bodyrates.y() = input_bounded(INPUT::kRateY);
  command.bodyrates.z() = input_bounded(INPUT::kRateZ);
  command.orientation.w() = state(STATE::kOriW);
  command.orientation.x() = state(STATE::kOriX);
  command.orientation.y() = state(STATE::kOriY);
  command.orientation.z() = state(STATE::kOriZ);
*/

/*
    // Compute dt
    float dt = ros::Time::now().toSec() - last_state_time_; // (float)(1.0f/ATTITUDE_RATE);
    last_state_time_ = ros::Time::now().toSec();
    // std::cout << "dt: " << dt << std::endl;

    // Position and Velocity error
    Vector3d p_error = sp_pos_ - pos_;
    Vector3d v_error = sp_vel_ - vel_;
    // std::cout << "p_error: " << p_error << std::endl;
    // std::cout << "v_error: " << v_error << std::endl;

    // Integral Error
    i_error_x += p_error(0) * dt;
    i_error_x = std::max(std::min(p_error(0), i_range_xy), -i_range_xy);

    i_error_y += p_error(1) * dt;
    i_error_y = std::max(std::min(p_error(1), i_range_xy), -i_range_xy);

    i_error_z += p_error(2) * dt;
    i_error_z = std::max(std::min(p_error(2), i_range_z), -i_range_z);

    // Desired thrust [F_des]
    Vector3d target_thrust = Vector3d::Zero();
    Vector3d fb_thrust = Vector3d::Zero();

    fb_thrust(0) = kp_xy * p_error(0) + kd_xy * v_error(0) + ki_xy * i_error_x;
    fb_thrust(1) = kp_xy * p_error(1) + kd_xy * v_error(1) + ki_xy * i_error_y;
    fb_thrust(2) = kp_z  * p_error(2) + kd_z  * v_error(2) + ki_z  * i_error_z;

    target_thrust(0) = sp_acc_(0);
    target_thrust(1) = sp_acc_(1);
    target_thrust(2) = (sp_acc_(2) + GRAVITY_MAGNITUDE);

    target_thrust = target_thrust + fb_thrust;

    // std::cout << "target_thrust: " << target_thrust << std::endl;

    // Move YAW angle setpoint
    double yaw_rate = 0;
    double yaw_des = sp_yaw_;

    // Z-Axis [zB]
    Matrix3d R = quat_.toRotationMatrix();
    Vector3d z_axis = R.col(2);

    // Current thrust [F]
    double current_thrust = target_thrust.dot(z_axis);

    // Calculate axis [zB_des]
    Vector3d z_axis_desired = target_thrust.normalized();

    // [xC_des]
    // x_axis_desired = z_axis_desired x [sin(yaw), cos(yaw), 0]^T
    Vector3d x_c_des;
    x_c_des(0) = cosf(radians(yaw_des));
    x_c_des(1) = sinf(radians(yaw_des));
    x_c_des(2) = 0;

    // [yB_des]
    // Vector3d y_axis_desired = (z_axis_desired.cross(x_c_des)).normalized();
    Vector3d y_axis_desired = (z_axis_desired.cross(R.col(0))).normalized();

    // [xB_des]
    Vector3d x_axis_desired = (y_axis_desired.cross(z_axis_desired)).normalized();

    Matrix3d Rdes;
    Rdes.col(0) = x_axis_desired;
    Rdes.col(1) = y_axis_desired;
    Rdes.col(2) = z_axis_desired;

    testbed_msgs::ControlStamped control_msg;

    switch (ctrl_mode_) {
      // Control the drone with attitude commands
      case ControlMode::ANGLES: 
        {
          // Create "Heading" rotation matrix (x-axis aligned w/ drone but z-axis vertical)
          Matrix3d Rhdg;
          Vector3d x_c(R(0,0) ,R(1,0), 0);
          x_c.normalize();
          Vector3d z_c(0, 0, 1);
          Vector3d y_c = z_c.cross(x_c);
          Rhdg.col(0) = x_c;
          Rhdg.col(1) = y_c;
          Rhdg.col(2) = z_c;

          Matrix3d Rout = Rhdg.transpose() * Rdes;

          Matrix3d Rerr = 0.5 * (Rdes.transpose() * Rhdg - Rhdg.transpose() * Rdes);
          Vector3d Verr(-Rerr(1,2),Rerr(0,2),-Rerr(0,1));

          // std::cout << "Rout: " << Rout << std::endl;

          control_msg.header.stamp = ros::Time::now();

          control_msg.control.roll = std::atan2(Rout(2,1),Rout(2,2));
          control_msg.control.pitch = -std::asin(Rout(2,0));
          control_msg.control.yaw_dot = -10*std::atan2(R(1,0),R(0,0)); //std::atan2(Rdes(1,0),Rdes(0,0));
          control_msg.control.thrust = current_thrust;
          break; 
        }
        // Control the drone with rate commands
      case ControlMode::RATES:
        {
          // Compute the rotation error between the desired z_ and the current one in Inertial frame
          Vector3d ni = z_axis.cross(z_axis_desired);
          double alpha = std::acos(ni.norm());
          ni.normalize();

          // Express the axis in body frame
          Vector3d nb = quat_.inverse() * ni;
          Quaterniond q_pq(Eigen::AngleAxisd(alpha, nb));

          control_msg.control.roll = (q_pq.w() > 0) ? (2.0 * kpq_rates_ * q_pq.x()) : (-2.0 * kpq_rates_ * q_pq.x());
          control_msg.control.pitch = (q_pq.w() > 0) ? (2.0 * kpq_rates_ * q_pq.y()) : (-2.0 * kpq_rates_ * q_pq.y());

          Quaterniond q_r = q_pq.inverse() * quat_.inverse() * Quaterniond(Rdes);
          control_msg.control.yaw_dot = (q_r.w() > 0) ? (2.0 * kr_rates_ * q_r.z()) : (-2.0 * kr_rates_ * q_r.z());
          control_msg.control.yaw_dot = -1.0 * control_msg.control.yaw_dot;
          ROS_ERROR("HEREHERE");

          break;
        }
        // Something is wrong if Default...
              default:
        ROS_ERROR("%s: Unable to select the control mode.", name_.c_str());
    }
*/
    control_pub_.publish(control_msg);
  }

}
