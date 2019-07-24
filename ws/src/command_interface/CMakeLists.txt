cmake_minimum_required(VERSION 2.8.3)
project(command_interface)

find_package(catkin REQUIRED 
	COMPONENTS 
	message_generation 
	rospy
	testbed_msgs
	testbed_srvs
	)


catkin_package()
