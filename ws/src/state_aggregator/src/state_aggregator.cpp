#include "state_aggregator/state_aggregator.hpp"


// HELPER FUNCTIONS
static double time_diff(timespec t1, timespec t2) {
    long int nsec = (t1.tv_nsec - t2.tv_nsec);
    long int sec = (t1.tv_sec - t2.tv_sec);
    if ( nsec < 0) {
        sec -= 1;
        nsec = 1e9 + nsec;
    } 		

    return (double)(sec + (double)nsec/1e9); 
}


static Quaterniond qsum(Quaterniond q1, Quaterniond q2) {
    Quaterniond res(Quaterniond::Identity());

    res.x() = q1.x() + q2.x();
    res.y() = q1.y() + q2.y();
    res.z() = q1.z() + q2.z();
    res.w() = q1.w() + q2.w();

    return res;
}

static Quaterniond qsm(Quaterniond q1, double a) {
    Quaterniond res(Quaterniond::Identity());

    res.x() = q1.x() * a;
    res.y() = q1.y() * a;
    res.z() = q1.z() * a; 
    res.w() = q1.w() * a;

    return res;
}

static Vector3d qd2w(Quaterniond q, Quaterniond qd) {
    Vector3d out;
    Matrix<double, 3, 4> M; 
    Vector4d v;
    v.block<3,1>(0,0) = qd.vec();
    v(3) = qd.w();
    M << -q.x(), q.w(), q.z(), -q.y(),
      -q.y(), -q.z(), q.w(), q.x(),
      -q.z(), q.y(), -q.x(), q.w();

    out = M * v;
    return out;
}


// =================================================================
// CLASS
//
StateAggregator::StateAggregator():
    received_reference_(false),
    last_state_time_(-1.0),
    initialized_(false) {};

StateAggregator::~StateAggregator() {};

bool StateAggregator::Initialize(const ros::NodeHandle& n) {

    ros::NodeHandle nl(n);

    // Compose the name
    name_ = ros::this_node::getName().c_str();

    // Load parameters
    if (!LoadParameters(nl)) {
        ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
        return false;
    }

    // Register callback
    if (!RegisterCallbacks(nl)) {
        ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
        return false;
    }

    // Advertise topics
    ext_pos_pub_ = 
        nl.advertise<geometry_msgs::PointStamped> ("external_position", 3);
    pose_pub_ = 
        nl.advertise<geometry_msgs::PoseStamped> ("external_pose", 3);
    pose_rpy_pub_ =
        nl.advertise<geometry_msgs::Vector3Stamped> ("external_pose_rpy", 3);
    odometry_pub_ =
        nl.advertise<nav_msgs::Odometry> ("opti_odom", 3); 


    // Initialize the header refereces of odometry messages
    ext_odom_trans_.header.frame_id = "world";
    ext_odom_trans_.child_frame_id = "cf1";

    // Initialize the header part of the odometry topic message
    ext_odometry_msg_.header.frame_id = "world";
    ext_odometry_msg_.child_frame_id = "cf1";

    bool initialized_ = true;

    return true;
}

bool StateAggregator::LoadParameters(const ros::NodeHandle& n) {

    ros::NodeHandle np("~");

    np.param<std::string>("topic", state_topic_, "/vrpn_client_node/cf1/pose");
    
//    ROS_INFO("Namespace = %s", );
    // Params
    std::string key;
    if (np.searchParam("valpha", key)) {
            ROS_INFO("Found parameter %s!", key.c_str());
            n.getParam(key, v_alpha_);
    } else {
            v_alpha_ = 0.7;
            ROS_INFO("No param 'valpha' found!"); 
            ROS_INFO("Setting default parameter %s = %f", 
                            "valpha", v_alpha_);
    }

    if (np.searchParam("qdalpha", key)) {
            ROS_INFO("Found parameter %s!", key.c_str());
            n.getParam(key, qd_alpha_);
    } else {
            qd_alpha_ = 0.7;
            ROS_INFO("No param 'qdalpha' found!"); 
            ROS_INFO("Setting default parameter %s = %f", 
                            "qd_alpha", qd_alpha_);
    }

    if (np.searchParam("time_delay", key)) {
            ROS_INFO("Found parameter %s!", key.c_str());
            n.getParam(key, t_delay_);
    } else {
            t_delay_ = 0.0;
            ROS_INFO("No param 'time_delay' found!"); 
            ROS_INFO("Setting default parameter %s = %f", 
                            "time_delay", t_delay_);

    }
    return true;
}


bool StateAggregator::RegisterCallbacks(const ros::NodeHandle& n) {
    ros::NodeHandle nl(n);
    inchannel1_= nl.subscribe(state_topic_, 10, &StateAggregator::onNewPose, this);	
    return true;
}


// CALLBACK ---------------------------------------------------------------------
/* 
 * Topic Callback
 * Whenever I receive a new Trajectory message, update the odometry.
 */
void StateAggregator::onNewPose(
        const boost::shared_ptr<geometry_msgs::PoseStamped const>& msg) {

    double dt;
    static timespec t_old;
    timespec t;

    // Take the time
    ros::Time current_time = ros::Time::now();

    // Fetch the ROS message
    p_(0) = msg->pose.position.x;	
    p_(1) = msg->pose.position.y;	
    p_(2) = msg->pose.position.z;	

    q_.x() = msg->pose.orientation.x;
    q_.y() = msg->pose.orientation.y;
    q_.z() = msg->pose.orientation.z;
    q_.w() = msg->pose.orientation.w;

    t.tv_sec = msg->header.stamp.sec;
    t.tv_nsec = msg->header.stamp.nsec;

    if (received_reference_) {
        p_old_ = p_;
        q_old_= q_;
        t_old = t; 

        received_reference_ = true;
    } else {
        dt = time_diff(t, t_old); 
        t_old = t;

        vel_ = (1.0 - v_alpha_) * vel_ + v_alpha_ * (p_ - p_old_)/dt;
        qd_ = qsum(qsm(qd_, (1.0 - qd_alpha_)), 
                qsm(qsum(q_, qsm(q_old_, -1)), qd_alpha_/dt));
        w_ = qd2w(q_, qd_);

        p_old_ = p_;
        q_old_ = q_;
    }

    // Push forward the position and attitude, considering
    // the delay
    if (t_delay_ > 0.0) {
        p_pf_ = p_ + vel_ * t_delay_;
        q_pf_ = qsum(q_, qsm(qd_, t_delay_));
        q_pf_ = q_pf_.normalized();
    }

    // Convert to euler
    euler_ = q_pf_.toRotationMatrix().eulerAngles(0, 1, 2);

    // Pose: Position + Orientation
    ext_pose_msg_.pose.position.x = p_pf_(0);
    ext_pose_msg_.pose.position.y = p_pf_(1);
    ext_pose_msg_.pose.position.z = p_pf_(2);

    ext_pose_msg_.pose.orientation.x = q_pf_.x();
    ext_pose_msg_.pose.orientation.y = q_pf_.y();
    ext_pose_msg_.pose.orientation.z = q_pf_.z();
    ext_pose_msg_.pose.orientation.w = q_pf_.w();

    // Position
    ext_position_msg_.point = ext_pose_msg_.pose.position;

    ext_pose_rpy_msg_.vector.x = euler_(0) * 180.0 / M_PI;
    ext_pose_rpy_msg_.vector.y = euler_(1) * 180.0 / M_PI;
    ext_pose_rpy_msg_.vector.z = euler_(2) * 180.0 / M_PI;
    ext_pose_rpy_msg_.header.stamp = msg->header.stamp;

    // Odometry Topic
    ext_odometry_msg_.pose.pose.position = ext_pose_msg_.pose.position;
    ext_odometry_msg_.pose.pose.orientation = ext_pose_msg_.pose.orientation;
    ext_odometry_msg_.twist.twist.linear.x = vel_(0);
    ext_odometry_msg_.twist.twist.linear.y = vel_(1);
    ext_odometry_msg_.twist.twist.linear.z = vel_(2);
    ext_odometry_msg_.twist.twist.angular.x = w_(0);
    ext_odometry_msg_.twist.twist.angular.y = w_(1);
    ext_odometry_msg_.twist.twist.angular.z = w_(2);

    // Update Tranformation Message	
    ext_odom_trans_.header.stamp = msg->header.stamp;
    // Position
    ext_odom_trans_.transform.translation.x = p_pf_[0];
    ext_odom_trans_.transform.translation.y = p_pf_[1];
    ext_odom_trans_.transform.translation.z = p_pf_[2];
    // Orientation
    ext_odom_trans_.transform.rotation = ext_pose_msg_.pose.orientation;


    // Send messages and transorm
    ext_odom_broadcaster_.sendTransform(ext_odom_trans_);

    ext_pos_pub_.publish(ext_position_msg_);
    pose_pub_.publish(ext_pose_msg_);
    pose_rpy_pub_.publish(ext_pose_rpy_msg_);
    odometry_pub_.publish(ext_odometry_msg_);

    return;
}


