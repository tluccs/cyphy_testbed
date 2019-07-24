///////////////////////////////////////////////////////////////////////////////
//
// State aggregator, which fuses state information from different sources and 
// compose an overall estimate of the vehicle position. 
// 
//
///////////////////////////////////////////////////////////////////////////////

#ifndef STATE_AGGREGATOR_H
#define STATE_AGGREGATOR_H

#include <ros/ros.h>
#include <string>
#include <time.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include "Eigen/Dense"

using namespace Eigen;




// =================================================================
// CLASS
//
class StateAggregator {

        public:
                StateAggregator();
                ~StateAggregator();

                // Initialize this class by reading parameters and loading callbacks.
                bool Initialize(const ros::NodeHandle& n);

                // Callback on Pose 
                void onNewPose(
                                const boost::shared_ptr<geometry_msgs::PoseStamped const>& msg);

        private:

                // Load parameters and register callbacks.
                bool LoadParameters(const ros::NodeHandle& n);
                bool RegisterCallbacks(const ros::NodeHandle& n);

                // K matrix and reference state/control (to fight gravity). These are

                // Remember last time we got a state callback.
                double last_state_time_;

                // Publishers and subscribers.
                // COMM ------------------------------------------------------------
                // Output publishers and broadcaster:
                ros::Publisher ext_pos_pub_;
                ros::Publisher pose_pub_;
                ros::Publisher pose_rpy_pub_;
                ros::Publisher odometry_pub_;
                tf::TransformBroadcaster ext_odom_broadcaster_;

                ros::Subscriber inchannel1_; 

                // Topics names
                std::string state_topic_;
                std::string control_topic_;

                // Initialized flag and name.
                bool received_reference_;
                bool initialized_;
                std::string name_;

                // DATA ------------------------------------------------------------
                // Pubblication variables 
                geometry_msgs::PoseStamped ext_pose_msg_;
                geometry_msgs::Vector3Stamped ext_pose_rpy_msg_;
                geometry_msgs::PointStamped ext_position_msg_;
                nav_msgs::Odometry ext_odometry_msg_;
                geometry_msgs::TransformStamped ext_odom_trans_;

                // ===========================================================
                // Helper variables
                Eigen::Vector3d p_;
                Eigen::Vector3d p_pf_;
                Eigen::Vector3d p_old_;
                Eigen::Vector3d vel_;
                Eigen::Vector3d euler_;
                Eigen::Vector3d w_;

                Eigen::Quaterniond q_;
                Eigen::Quaterniond q_old_;
                Eigen::Quaterniond qd_;
                Eigen::Quaterniond q_pf_;

                double t_delay_; 
                double v_alpha_, qd_alpha_;

}; //\class StateAggregator


#endif
