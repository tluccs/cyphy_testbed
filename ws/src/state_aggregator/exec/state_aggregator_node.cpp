
#include <ros/ros.h>
#include "state_aggregator/state_aggregator.hpp"


int main(int argc, char** argv) {

    ros::init(argc, argv, "state_aggregator");
    ros::NodeHandle nh;

    ROS_INFO("Starting State Aggregator Node");

    StateAggregator sa;
   
    if(!sa.Initialize(nh)) {
        ROS_ERROR("%s: Failed to initialize state_aggregator.",
              ros::this_node::getName().c_str());
        return EXIT_FAILURE;
    }

    ros::spin();

    return EXIT_SUCCESS;
}

