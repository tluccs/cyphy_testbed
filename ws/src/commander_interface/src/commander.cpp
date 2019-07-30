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
        ros::NodeHandle nh("~");
        // Get the name of the node
        name_ = ros::this_node::getName().c_str();

        // Load Parameters
        if (!LoadParameters(n)) {
                ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
                return false;
        }

        // Advertise topics/Services
        takeoff_srv_ = nh.advertiseService("takeoff_srv", 
                        &CommanderInterface::takeoff_callback, this);

        land_srv_ = nh.advertiseService("land_srv", 
                        &CommanderInterface::land_callback, this);

        goTo_srv_ = nh.advertiseService("goTo_srv",
                        &CommanderInterface::goto_callback, this);

        track_srv_ = nh.advertiseService("track_srv",
                        &CommanderInterface::track_callback, this);

        guidance_clnt_ = nh.serviceClient<guidance::GenTrackTrajectory>(
                                "gen_TrackTrajectory");


        initialized_ = true; 

        return true;
}


// Service Callbacks
bool CommanderInterface::takeoff_callback(
                commander_interface::TakeOff::Request  &req,
                commander_interface::TakeOff::Response &res) {
        guidance::GenTrackTrajectory srv;

        boost::array<float, 3> v{{0.0, 0.0, 0.0}};

        srv.request.target_v = v;
        srv.request.target_a = v;
        v[2] = req.height;
        srv.request.target_p = v; 

        srv.request.tg_time = req.duration;

        if (guidance_clnt_.call(srv))
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

        srv.request.target_v = v;
        srv.request.target_a = v;
        v[2] = req.height;
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

        if (guidance_clnt_.call(srv))
                res.ack = "Roger!";
        else
                res.ack = "Fail!";

        return true;
}
