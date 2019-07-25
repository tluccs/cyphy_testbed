cmake_minimum_required(VERSION 2.8.3)
project(monitors)

find_package(catkin REQUIRED 
        COMPONENTS 
        roscpp
        tf
        std_msgs
        message_generation
        testbed_msgs
        )

catkin_package(
        INCLUDE_DIRS include/ 
        CATKIN_DEPENDS roscpp tf std_msgs testbed_msgs
        )

include_directories(
        include/
        ${catkin_INCLUDE_DIRS}	
        )

add_executable(ctrl_setpoint_odometry_publisher
        exec/ctrl_setpoint_odometry_publisher_node.cpp  
        src/odometry_publisher.cpp
        )

add_dependencies(ctrl_setpoint_odometry_publisher 
        ${catkin_EXPORTED_TARGETS})

target_link_libraries(ctrl_setpoint_odometry_publisher
        ${catkin_LIBRARIES}
        )
