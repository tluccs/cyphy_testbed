/**
 * @file odometry_publisher.hpp
 * @author l.pannocchi@gmail.com
 *
 */
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "testbed_msgs/ControlSetpoint.h"

class OdometryPublisher {
        public:
                OdometryPublisher();
                ~OdometryPublisher();

                bool Initialize(const ros::NodeHandle& n);

                // Callback
                void callback(const boost::shared_ptr<testbed_msgs::ControlSetpoint const>& msg);

        private:
                bool LoadParameters(const ros::NodeHandle& n);
                bool RegisterCallbacks(const ros::NodeHandle& n);

                double trj_pos[3];
                double trj_vel[3];
                double trj_acc[3];
                double euler[3];

                // Input
                ros::Subscriber inchannel1_; 

                // Output
                ros::Publisher odom_pub_;
                tf::TransformBroadcaster odom_broadcaster_; 

                // Topics
                std::string ghost_topic_;
                std::string ctrlsetpoint_topic_;

                // Odometry: TF 
                geometry_msgs::TransformStamped odom_trans_;
                // Odometry Topic
                nav_msgs::Odometry odom_;

                // Node name
                std::string name_;
                std::string cm_id_;
                std::string tg_frame_id_;
                

                bool initialized_;
};


