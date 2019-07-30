#ifndef COMMANDER_INTERFACE_H
#define COMMANDER_INTERFACE_H

#include "ros/ros.h"

#include "commander_interface/TakeOff.h"
#include "commander_interface/Land.h"
#include "commander_interface/Track.h"
#include "commander_interface/GoTo.h"

// =================================================================
// CLASS
//

class CommanderInterface {
        public:
                CommanderInterface();
                ~CommanderInterface();

                bool Initialize(const ros::NodeHandle& n);

                bool takeoff_callback( 
                                commander_interface::TakeOff::Request  &req,
                                commander_interface::TakeOff::Response &res);
                bool land_callback(
                                commander_interface::Land::Request  &req,
                                commander_interface::Land::Response &res);

                bool goto_callback(
                                commander_interface::GoTo::Request  &req,
                                commander_interface::GoTo::Response &res);

                bool track_callback(
                                commander_interface::Track::Request  &req,
                                commander_interface::Track::Response &res);

        private:

                // Load Parameters
                bool LoadParameters(const ros::NodeHandle& n);
//                bool RegisterCallbacks(const ros::NodeHandle& n);

                // ROS variables
                //
                // Service Server
                
                ros::ServiceServer takeoff_srv_;
                ros::ServiceServer land_srv_;
                ros::ServiceServer goTo_srv_;
                ros::ServiceServer track_srv_;
                // Service Client
                ros::ServiceClient guidance_clnt_;



                // Callbacks


                // Names and topics
                std::string name_;

                bool initialized_;
};

#endif
