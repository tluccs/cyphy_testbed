cmake_minimum_required(VERSION 2.8.3)
project(controller)

find_package(catkin REQUIRED 
	COMPONENTS
	message_generation
	roscpp
	tf
	std_msgs
	testbed_msgs
)

find_package (Eigen3 REQUIRED NO_MODULE)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS 
	roscpp
	message_runtime
	tf
	std_msgs
	testbed_msgs
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_executable(mellinger_controller_node 
	exec/mellinger_controller_node.cpp 
	src/mellinger_controller.cpp
)

add_dependencies(mellinger_controller_node crazyflie_demo_generate_messages_cpp)

target_link_libraries(
	mellinger_controller_node
	Eigen3::Eigen 
	${catkin_LIBRARIES}
)