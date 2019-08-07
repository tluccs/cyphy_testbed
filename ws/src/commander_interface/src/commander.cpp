/** @file commander.cpp
 *  @author l.pannocchi@gmail.com
 *
 */
#include "commander/commander.hpp"
#include "guidance/GenTrackTrajectory.h"

// =================================================================
// =================================================================
CommanderInterface::CommanderInterface() :
    takeoff_srv_(), land_srv_(), goTo_srv_(), track_srv_()
{
        initialized_ = false;
}

CommanderInterface::~CommanderInterface() {
        return;
}


bool CommanderInterface::LoadParameters(const ros::NodeHandle& n) {
        return true;
}

/**
 * Initialization function
 */
bool CommanderInterface::Initialize(const ros::NodeHandle& n) {
        // Get the name of the node
        name_ = ros::this_node::getName();
        namespace_ = ros::this_node::getNamespace();

        // Load Parameters
        if (!LoadParameters(n)) {
                ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
                return false;
        }

        // I want to advertise service in the node namespace
        ros::NodeHandle nh("~");
        ros::NodeHandle ng(n);

        // Advertise topics/Services
        takeoff_srv_ = nh.advertiseService("takeoff_srv", 
                        &CommanderInterface::takeoff_callback, this);

        land_srv_ = nh.advertiseService("land_srv", 
                        &CommanderInterface::land_callback, this);

        goTo_srv_ = nh.advertiseService("goTo_srv",
                        &CommanderInterface::goto_callback, this);

        track_srv_ = nh.advertiseService("track_srv",
                        &CommanderInterface::track_callback, this);

        // Connect to the service provided by the guidance node.
        guidance_clnt_ = ng.serviceClient<guidance::GenTrackTrajectory>(
                                "gen_TrackTrajectory");


        initialized_ = true; 

        return true;
}


// Service Callbacks
bool CommanderInterface::takeoff_callback(
                commander_interface::TakeOff::Request  &req,
                commander_interface::TakeOff::Response &res) {

        ROS_INFO("%s: Takeoff requested! \n \t Height: %.3f | Duration: %.3f", 
                        name_.c_str(), req.height, req.duration);

        guidance::GenTrackTrajectory srv;

        boost::array<float, 3> v{{0.0, 0.0, 0.0}};

        srv.request.ref = "Relative";
        srv.request.target_v = v;
        srv.request.target_a = v;
        v[2] = req.height;
        srv.request.target_p = v; 

        srv.request.tg_time = req.duration;

        if (guidance_clnt_.call(srv) == true)
                res.ack = "Roger!";
        else
                res.ack = "Fail!";

        return true;
}

bool CommanderInterface::land_callback(
                commander_interface::Land::Request  &req,
                commander_interface::Land::Response &res) {

        guidance::GenTrackTrajectory srv;

        boost::array<float, 3> v{{0.0, 0.0, 0.0}};
        
        // Relative request to the current point
        srv.request.ref = "Relative";
        v[2] = -10.0; // Dirty, but it will work in practice
        srv.request.target_p = v; 

        srv.request.tg_time = req.duration;

        if (guidance_clnt_.call(srv))
                res.ack = "Roger!";
        else
                res.ack = "Fail!";

        return true;
}

bool CommanderInterface::goto_callback(
                commander_interface::GoTo::Request  &req,
                commander_interface::GoTo::Response &res) {

        guidance::GenTrackTrajectory srv;

        boost::array<float, 3> v{{0.0, 0.0, 0.0}};

        srv.request.target_v = v;
        srv.request.target_a = v;

        v = req.target_p;
        srv.request.target_p = v; 

        srv.request.tg_time = req.duration;

        if (guidance_clnt_.call(srv))
                res.ack = "Roger!";
        else
                res.ack = "Fail!";

        return true;
}

bool CommanderInterface::track_callback(
                commander_interface::Track::Request  &req,
                commander_interface::Track::Response &res) {

        guidance::GenTrackTrajectory srv;

        srv.request.target_p = req.target_p; 
        srv.request.target_v = req.target_v;
        srv.request.target_a = req.target_a;
        srv.request.tg_time = req.duration;
        srv.request.ref = "Absolute";

        if (guidance_clnt_.call(srv))
                res.ack = "Roger!";
        else
                res.ack = "Fail!";

        return true;
}
