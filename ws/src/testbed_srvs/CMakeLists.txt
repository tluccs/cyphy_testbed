cmake_minimum_required(VERSION 2.8.6)
project("testbed_srvs")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
	roscpp
	message_generation
	)


file(GLOB srv_files RELATIVE 
	${PROJECT_SOURCE_DIR}/srv ${PROJECT_SOURCE_DIR}/srv/*.srv)

add_service_files(
	DIRECTORY srv 
	FILES ${srv_files}
	)

generate_messages(
	)

catkin_package(
	CATKIN_DEPENDS
	roscpp
	message_runtime
	)

