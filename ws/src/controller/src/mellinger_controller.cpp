#include <ros/ros.h>
#include "mellinger_controller.h"
#include <math.h>
#include "math3d.h"
#include <stdio.h>

#define GRAVITY_MAGNITUDE (9.81f)

namespace controller {

// Initialize.
bool MellingerController::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "controller");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  // // Load K, x_ref, u_ref from disk.
  // if (!LoadFromDisk()) {
  //   ROS_ERROR("%s: Failed to load K, x_ref, u_ref from disk.", name_.c_str());
  //   return false;
  // }

  // Set up control publisher.
  ros::NodeHandle nl(n);
  control_pub_ = nl.advertise<testbed_msgs::ControlStamped>(
    control_topic_.c_str(), 1, false);

  Vector3d sp_pos_ = Vector3d::Zero();
  Vector3d sp_vel_ = Vector3d::Zero();
  Vector3d sp_acc_ = Vector3d::Zero();
  Vector3d sp_r_pos_ = Vector3d::Zero();
  Vector3d sp_r_vel_ = Vector3d::Zero();
  Vector3d sp_r_acc_ = Vector3d::Zero();

  Vector3d pos_ = Vector3d::Zero();
  Vector3d vel_ = Vector3d::Zero();
  Vector3d r_pos_ = Vector3d::Zero();
  Vector3d r_vel_ = Vector3d::Zero();

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

void MellingerController::Reset(void)
{
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

  // sp_r_pos_(0) = msg->setpoint.roll;
  // sp_r_pos_(1) = msg->setpoint.pitch;
  // sp_r_pos_(2) = msg->setpoint.yaw;

  // sp_r_vel_(0) = msg->setpoint.roll_dot;
  // sp_r_vel_(1) = msg->setpoint.pitch_dot;
  // sp_r_vel_(2) = msg->setpoint.yaw_dot;

  // sp_r_acc_(0) = msg->setpoint.roll_ddot;
  // sp_r_acc_(1) = msg->setpoint.pitch_ddot;
  // sp_r_acc_(2) = msg->setpoint.yaw_ddot;

  received_setpoint_ = true;
}

// Process an incoming state measurement.
void MellingerController::StateCallback(
  const nav_msgs::Odometry::ConstPtr& msg) {
  // Catch no setpoint.
  if (!received_setpoint_)
    return;

  if (last_state_time_ < 0.0)
    last_state_time_ = ros::Time::now().toSec();

  // Read the message into the state and compute relative state.
  // VectorXd x(x_dim_);
  pos_(0) = msg->pose.pose.position.x;
  pos_(1) = msg->pose.pose.position.y;
  pos_(2) = msg->pose.pose.position.z;

  vel_(0) = msg->twist.twist.linear.x;
  vel_(1) = msg->twist.twist.linear.y;
  vel_(2) = msg->twist.twist.linear.z;

  quat_.vec() = Vector3d (msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  quat_.w() = msg->pose.pose.orientation.w;
  quat_.normalize();

  //r_pos_(0) = msg->state.roll;
  //r_pos_(1) = msg->state.pitch;
  //r_pos_(2) = msg->state.yaw;

  //r_vel_(0) = msg->state.roll_dot;
  //r_vel_(1) = msg->state.pitch_dot;
  //r_vel_(2) = msg->state.yaw_dot;
  //std::cout << "quaternion: " << quat_.vec() << ", " << quat_.w() << std::endl;

  float dt = ros::Time::now().toSec() - last_state_time_; // (float)(1.0f/ATTITUDE_RATE);
  last_state_time_ = ros::Time::now().toSec();

  //std::cout << "dt: " << dt << std::endl;

  Vector3d p_error = sp_pos_ - pos_;
  Vector3d v_error = sp_vel_ - vel_;

  std::cout << "p_error: " << p_error << std::endl;
  std::cout << "v_error: " << v_error << std::endl;

  // Integral Error

  i_error_x += p_error(0) * dt;
  i_error_x = std::max(std::min(p_error(0), i_range_xy), -i_range_xy);

  i_error_y += p_error(1) * dt;
  i_error_y = std::max(std::min(p_error(1), i_range_xy), -i_range_xy);

  i_error_z += p_error(2) * dt;
  i_error_z = std::max(std::min(p_error(2), i_range_z), -i_range_z);

  // Desired thrust [F_des]
  Vector3d target_thrust = Vector3d::Zero();
  target_thrust(0) = sp_acc_(0)                      + kp_xy * p_error(0) + kd_xy * v_error(0);// + ki_xy * i_error_x;
  target_thrust(1) = sp_acc_(1)                      + kp_xy * p_error(1) + kd_xy * v_error(1);// + ki_xy * i_error_y;
  target_thrust(2) = (sp_acc_(2) + GRAVITY_MAGNITUDE) + kp_z  * p_error(2) + kd_z  * v_error(2);// + ki_z  * i_error_z;

  //std::cout << "target_thrust: " << target_thrust << std::endl;

  // Move YAW angle setpoint
  double desiredYaw = sp_yaw_;

  // // Z-Axis [zB]
  //Eigen::Quaterniond q;
  Matrix3d R = quat_.toRotationMatrix();
  Vector3d z_axis = R.col(2);

  //std::cout << "z_axis: " << z_axis << std::endl;

  // Current thrust [F]
  double current_thrust = target_thrust.dot(z_axis);

  // Calculate axis [zB_des]
  Vector3d z_axis_desired = target_thrust.normalized();

  // [xC_des]
  // x_axis_desired = z_axis_desired x [sin(yaw), cos(yaw), 0]^T
  Vector3d x_c_des;
  x_c_des(0) = cosf(radians(desiredYaw));
  x_c_des(1) = sinf(radians(desiredYaw));
  x_c_des(2) = 0;
  // [yB_des]
  Vector3d y_axis_desired = (z_axis_desired.cross(x_c_des)).normalized();
  // [xB_des]
  Vector3d x_axis_desired = (y_axis_desired.cross(z_axis_desired)).normalized();

  Matrix3d Rdes;
  Rdes.col(0) = x_axis_desired;
  Rdes.col(1) = y_axis_desired;
  Rdes.col(2) = z_axis_desired;

  //std::cout << "Rdes: " << Rdes << std::endl;

  testbed_msgs::ControlStamped control_msg;
  control_msg.control.roll = std::atan2(Rdes(2,1),Rdes(2,2));
  control_msg.control.pitch = -std::asin(Rdes(2,0));
  control_msg.control.yaw_dot = -std::atan2(R(1,0),R(0,0)); //std::atan2(Rdes(1,0),Rdes(0,0));
  control_msg.control.thrust = current_thrust;

  control_pub_.publish(control_msg);
}

}