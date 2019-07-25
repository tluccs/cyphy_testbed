/**
 * @file setpoint_ghost_pub.cpp
 *
 * @author l.pannocchi@gmail.com 
 *
 *
 * This ROS node subscribe to the topic regarding the generated 
 * control reference and converts the message into an odometry ROS 
 * message that can be plotted with rviz.
 *
 */

#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <testbed_msgs/ControlSetpoint.h>

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
// Odometry: TF 
static geometry_msgs::TransformStamped ctrlsetpoint_odom_trans;

// Odometry Topic
static nav_msgs::Odometry ctrlsetpoint_odom;


// CALLBACK ---------------------------------------------------------------------
void ctrlsetpoint_callback(const boost::shared_ptr<testbed_msgs::ControlSetpoint const>& msg);

// MAIN -------------------------------------------------------------------------
int main(int argc, char** argv) {

	ROS_INFO("Starting Control Setpoint publisher");

	// Initialize the ROS stack
	ros::init(argc, argv, "CtlSetpoint_publisher");
	ros::NodeHandle nh("~");

	double nodeRate;
    std::string frame; 
    std::string command_id;

	nh.param("ghostNrate", nodeRate, 100.0);
	nh.param<std::string>("frame", frame, "cf1");
    nh.param<std::string>("command_id", command_id, "cm1");

	ROS_INFO("Publishing at %.3f", nodeRate);

	// OUTPUTS:
	ros::Publisher ctrlsetpoint_odom_pub;
	tf::TransformBroadcaster ctrlsetpoint_odom_broadcaster;
    
	// 1) Advertise topic
	ctrlsetpoint_odom_pub = nh.advertise<nav_msgs::Odometry> ("/" + command_id + "/" + frame + "/ghost_vehicle_od", 20);

	// Initialize the header part of the odometry TF packet 
	ctrlsetpoint_odom_trans.header.frame_id = "world";
	ctrlsetpoint_odom_trans.child_frame_id = "/" + command_id + "_" + frame + "_ghost";

	// Initialize the header part of the odometry topic message
	ctrlsetpoint_odom.header.frame_id = "world";
	ctrlsetpoint_odom.child_frame_id = "/" + command_id + "_" + frame + "_ghost";

	// INPUT
	// Subscribe to the topic produced by the node sending the trajectory
	ros::Subscriber trj_sub = nh.subscribe("/" + command_id + "/" + frame + "/setpoint", 
			10, ctrlsetpoint_callback);	

	ros::Rate r(nodeRate);

	while (nh.ok()) {
		ros::spinOnce();
	
		// Send the transform
		ctrlsetpoint_odom_broadcaster.sendTransform(ctrlsetpoint_odom_trans);

		// Pubblish the odometry message
		ctrlsetpoint_odom_pub.publish(ctrlsetpoint_odom);
		r.sleep();
	}
}


/* 
 * Topic Callback
 * Whenever I receive a new Trajectory message, update the odometry.
 */
void ctrlsetpoint_callback(const boost::shared_ptr<testbed_msgs::ControlSetpoint const>& msg) {
	// Take the time
	ros::Time current_time = ros::Time::now();

	// Fetch the ROS message
	trj_pos[0] = msg->p.x;	
	trj_pos[1] = msg->p.y;	
	trj_pos[2] = msg->p.z;

	trj_vel[0] = msg->v.x;
	trj_vel[1] = msg->v.y;	
	trj_vel[2] = msg->v.z;	

	trj_acc[0] = msg->a.x;	
	trj_acc[1] = msg->a.y;
	trj_acc[2] = msg->a.z;

	euler[0] = msg->rpy.x;
	euler[1] = msg->rpy.y;
	euler[2] = msg->rpy.z;

	// Update Tranformation Message	
	ctrlsetpoint_odom_trans.header.stamp = current_time;
	// Position
	ctrlsetpoint_odom_trans.transform.translation.x = trj_pos[0];
	ctrlsetpoint_odom_trans.transform.translation.y = trj_pos[1];
	ctrlsetpoint_odom_trans.transform.translation.z = trj_pos[2];
	// Orientation
	ctrlsetpoint_odom_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(euler[0], euler[1], euler[2]);

	// Update Topic Message
	ctrlsetpoint_odom.header.stamp = current_time;
	// Pose part of the odometry message
	ctrlsetpoint_odom.pose.pose.position.x = trj_pos[0];
	ctrlsetpoint_odom.pose.pose.position.y = trj_pos[1];
	ctrlsetpoint_odom.pose.pose.position.z = trj_pos[2];
	ctrlsetpoint_odom.pose.pose.orientation = ctrlsetpoint_odom_trans.transform.rotation;
	// Twist part of the odometry message 
	ctrlsetpoint_odom.twist.twist.linear.x = trj_vel[0];
	ctrlsetpoint_odom.twist.twist.linear.y = trj_vel[1];
	ctrlsetpoint_odom.twist.twist.linear.z = trj_vel[2];
}

