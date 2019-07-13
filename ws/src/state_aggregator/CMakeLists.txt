cmake_minimum_required(VERSION 2.8.3)

project(state_aggregator_pkg)

# Find required packages
find_package(
    catkin REQUIRED COMPONENTS roscpp tf std_msgs
    )

catkin_package(
    INCLUDE_DIRS include/ # The package contains header files
    CATKIN_DEPENDS roscpp tf std_msgs
    )

# Since we use eigen
include_directories(
    include/
    /home/rt-2pm2/Extensions/eigen/
    ${catkin_INCLUDE_DIRS} # Found in "find_package"
)

# Declare the executable to be build and the associated source
add_executable(state_aggregator 
    exec/state_aggregator_node.cpp 
    src/state_aggregator.cpp)

target_link_libraries(state_aggregator
    ${catkin_LIBRARIES}
    )
