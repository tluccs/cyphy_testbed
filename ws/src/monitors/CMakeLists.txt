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
	CATKIN_DEPENDS roscpp tf std_msgs testbed_msgs
	)

include_directories(
	${catkin_INCLUDE_DIRS}	
	)

add_executable(setpoint_ghost_pub
	src/setpoint_ghost_pub.cpp)

add_dependencies(setpoint_ghost_pub 
	${catkin_EXPORTED_TARGETS})

target_link_libraries(setpoint_ghost_pub
	${catkin_LIBRARIES}
	)
