#include "odometry_publisher/odometry_publisher.hpp"

OdometryPublisher::OdometryPublisher() {
    for (int i = 0; i < 3; i++) {
            trj_pos[i] = 0.0;
            trj_vel[i] = 0.0;
            trj_acc[i] = 0.0;
            euler[i] = 0.0;
    }
}

OdometryPublisher::~OdometryPublisher() {
};

bool OdometryPublisher::Initialize(const ros::NodeHandle& n) {
        ros::NodeHandle nh(n);

        // Get node name
        name_ = ros::this_node::getName().c_str();

        // Load Parameters
        if (!LoadParameters(nh)) {
                ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
                return false;
        }

        // Register callback
        if (!RegisterCallbacks(nh)) {
                ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
                return false;
        }

        // Advertise topics
        ROS_INFO("Node %s: Publishing to %s", name_.c_str(), ghost_topic_.c_str());
        odom_pub_ = nh.advertise<nav_msgs::Odometry> (ghost_topic_, 20);


        // Initialize the header part of the odometry TF packet 
        odom_trans_.header.frame_id = "world";
        odom_trans_.child_frame_id =  "ghost";

        // Initialize the header part of the odometry topic message
        odom_.header.frame_id = "world";
        odom_.child_frame_id = "ghost";

        bool initialized_ = true;

        return true;
}

bool OdometryPublisher::LoadParameters(const ros::NodeHandle& n) {
        ros::NodeHandle nh("~");

        nh.param<std::string>("commander_id", cm_id_, "cm1");
        nh.param<std::string>("tg_frame_id", tg_frame_id_, "cf1");

        nh.param<std::string>("out_ghost_topic", ghost_topic_, "ghost");
        nh.param<std::string>("controlsetpoint_topic", ctrlsetpoint_topic_, "cf1/setpoint");

        ROS_INFO("Commander ID: %s", cm_id_.c_str());
        ROS_INFO("Target Frame: %s", tg_frame_id_.c_str());

        return true;
}

bool OdometryPublisher::RegisterCallbacks(const ros::NodeHandle& n) {
        ros::NodeHandle nh(n);
        ROS_INFO("Node %s: Subscribing to %s", name_.c_str(), ghost_topic_.c_str());
        inchannel1_ = nh.subscribe(ctrlsetpoint_topic_, 10, 
                        &OdometryPublisher::callback, this);	

        return true;
}

void OdometryPublisher::callback(const boost::shared_ptr<testbed_msgs::ControlSetpoint const>& msg) {
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
        odom_trans_.header.stamp = current_time;
        // Position
        odom_trans_.transform.translation.x = trj_pos[0];
        odom_trans_.transform.translation.y = trj_pos[1];
        odom_trans_.transform.translation.z = trj_pos[2];
        // Orientation
        odom_trans_.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(euler[0], euler[1], euler[2]);

        // Update Topic Message
        odom_.header.stamp = current_time;
        // Pose part of the odometry message
        odom_.pose.pose.position.x = trj_pos[0];
        odom_.pose.pose.position.y = trj_pos[1];
        odom_.pose.pose.position.z = trj_pos[2];
        odom_.pose.pose.orientation = odom_trans_.transform.rotation;
        // Twist part of the odometry message 
        odom_.twist.twist.linear.x = trj_vel[0];
        odom_.twist.twist.linear.y = trj_vel[1];
        odom_.twist.twist.linear.z = trj_vel[2];

        // Send the transform
        odom_broadcaster_.sendTransform(odom_trans_);
        // Pubblish the odometry message
        odom_pub_.publish(odom_);

        return;
}



