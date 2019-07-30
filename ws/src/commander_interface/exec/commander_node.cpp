#include "commander/commander.hpp"

int main(int argc, char* argv[]) {
        ros::init(argc, argv, "commander");
        ros::NodeHandle n("~");

        CommanderInterface commander;

        
        if (!commander.Initialize(n)) {
                ROS_ERROR("%s: Failed to initialize!", 
                                ros::this_node::getName().c_str());
                return EXIT_FAILURE;
        }

        ros::spin();

        return EXIT_SUCCESS;
}
