/**
 * @file publish_odometry.cpp
 *
 * @author rt-2pm2
 *
 *
 * This ROS node subscribe to the topic regarding the generated 
 * trajectory and * converts the message into an odometry ROS 
 * message that can be plotted with rviz.
 *
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <testbed_msgs/Setpoint.h>

// DATA -------------------------------------------------------------------------
// Pubblication variables 
//
// Odometry: Topic 
static float trj_pos[3];
static float trj_vel[3];
static float trj_acc[3];
static float euler[3];

// =======================================================
// ------------------------ GHOST ------------------------
// Odometry: quaternion
static geometry_msgs::Quaternion ghost_odom_quat;

// Odometry: TF 
static geometry_msgs::TransformStamped ghost_odom_trans;

// Odometry Topic
static nav_msgs::Odometry ghost_odom;


// CALLBACK ---------------------------------------------------------------------
void trj_callback(const boost::shared_ptr<testbed_msgs::Setpoint const>& msg);

// MAIN -------------------------------------------------------------------------
int main(int argc, char** argv) {

	ROS_INFO("Starting Ghost odometry publisher");

	// Initialize the ROS stack
	ros::init(argc, argv, "odometry_publisher");
	ros::NodeHandle nh;

	double nodeRate;
	nh.param("ghostNrate", nodeRate, 100.0);

	ROS_INFO("Publishing at %.3f", nodeRate);

	// OUTPUTS:
	ros::Publisher ghost_odom_pub;
	tf::TransformBroadcaster ghost_odom_broadcaster;
    
	// 1) Advertise topic
	ghost_odom_pub = nh.advertise<nav_msgs::Odometry> ("ghost_vehicle_od", 20);

	// Initialize the header part of the odometry TF packet 
	ghost_odom_trans.header.frame_id = "world";
	ghost_odom_trans.child_frame_id = "ghost_cf1";

	// Initialize the header part of the odometry topic message
	ghost_odom.header.frame_id = "world";
	ghost_odom.child_frame_id = "ghost_cf1";

	// INPUT
	// Subscribe to the topic produced by the node sending the trajectory
	ros::Subscriber trj_sub = nh.subscribe("/setpoint", 
			10, trj_callback);	

	ros::Rate r(nodeRate);

	while (nh.ok()) {
		ros::spinOnce();
	
		// Send the transform
		ghost_odom_broadcaster.sendTransform(ghost_odom_trans);

		// Pubblish the odometry message
		ghost_odom_pub.publish(ghost_odom);
		r.sleep();
	}
}


/* 
 * Topic Callback
 * Whenever I receive a new Trajectory message, update the odometry.
 */
void trj_callback(const boost::shared_ptr<testbed_msgs::Setpoint const>& msg) {
	// Take the time
	ros::Time current_time = ros::Time::now();

	// Fetch the ROS message
	trj_pos[0] = msg->px;	
	trj_pos[1] = msg->py;	
	trj_pos[2] = msg->pz;	

	trj_vel[0] = msg->vx;	
	trj_vel[1] = msg->vy;	
	trj_vel[2] = msg->vz;	

	trj_acc[0] = msg->accx;	
	trj_acc[1] = msg->accy;	
	trj_acc[2] = msg->accz;	

	euler[0] = msg->r;
	euler[1] = msg->p;
	euler[2] = msg->y;

	// Update Tranformation Message	
	ghost_odom_trans.header.stamp = current_time;
	// Update the odometry transformation packet with
	// the information received via ROS topic
	ghost_odom_quat = tf::createQuaternionMsgFromRollPitchYaw(euler[0], 
			euler[1], euler[2]);

	// Position
	ghost_odom_trans.transform.translation.x = trj_pos[0];
	ghost_odom_trans.transform.translation.y = trj_pos[1];
	ghost_odom_trans.transform.translation.z = trj_pos[2];
	// Orientation
	ghost_odom_trans.transform.rotation = ghost_odom_quat;

	// Update Topic Message
	ghost_odom.header.stamp = current_time;
	// Pose part of the odometry message
	ghost_odom.pose.pose.position.x = trj_pos[0];
	ghost_odom.pose.pose.position.y = trj_pos[1];
	ghost_odom.pose.pose.position.z = trj_pos[2];
	ghost_odom.pose.pose.orientation = ghost_odom_quat;

	// Twist part of the odometry message 
	ghost_odom.twist.twist.linear.x = trj_vel[0];
	ghost_odom.twist.twist.linear.y = trj_vel[1];
	ghost_odom.twist.twist.linear.z = trj_vel[2];

}

