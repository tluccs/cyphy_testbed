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

add_executable(cmd_vel_converter_node 
	exec/cmd_vel_converter_node.cpp 
	src/cmd_vel_converter.cpp
)

add_dependencies(mellinger_controller_node
	${catkin_EXPORTED_TARGETS}
	)

add_dependencies(cmd_vel_converter_node 
	${catkin_EXPORTED_TARGETS}
	)

target_link_libraries(
	mellinger_controller_node
	Eigen3::Eigen 
	mpc_wrapper
	${catkin_LIBRARIES}
)

target_link_libraries(
	cmd_vel_converter_node
	Eigen3::Eigen 
	${catkin_LIBRARIES}
)

add_library(mpc_solver
  externals/qpoases/SRC/Bounds.cpp
  externals/qpoases/SRC/Constraints.cpp
  externals/qpoases/SRC/CyclingManager.cpp
  externals/qpoases/SRC/Indexlist.cpp
  externals/qpoases/SRC/MessageHandling.cpp
  externals/qpoases/SRC/QProblem.cpp
  externals/qpoases/SRC/QProblemB.cpp
  externals/qpoases/SRC/SubjectTo.cpp
  externals/qpoases/SRC/Utils.cpp
  externals/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  model/quadrotor_mpc_codegen/acado_qpoases_interface.cpp
  model/quadrotor_mpc_codegen/acado_integrator.c
  model/quadrotor_mpc_codegen/acado_solver.c
  model/quadrotor_mpc_codegen/acado_auxiliary_functions.c)

target_link_libraries(mpc_solver ${catkin_LIBRARIES})

target_include_directories(mpc_solver PUBLIC
  model/quadrotor_mpc_codegen/
  externals/qpoases
  externals/qpoases/INCLUDE
  externals/qpoases/SRC)

add_library(mpc_wrapper
  src/mpc_wrapper.cpp)

target_link_libraries(mpc_wrapper Eigen3::Eigen ${catkin_LIBRARIES})

target_link_libraries(mpc_wrapper
  mpc_solver)

