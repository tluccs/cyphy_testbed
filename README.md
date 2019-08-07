# Cyphy Testbed

## Overall Description
This repository represents a testing and development framework for application in the domain of cyberphysical systems. 

Currently, the "*Crazyflie*" drone by *Bitcraze* is the platform on which the test are run, but the system has been designed to include different platforms in the future. 
The core of the framework is based on *ROS*.


## Components
The framework has been organized in modules (*ROS packages*) for ease of maintenance.

Currently the following modules have been implemented:

1. **commander_interface**
This package provides the point of interaction between the vehicle and the user. The *commander\_node* advertises services for requesting task to the drone, such as goto, land, takeoff and full state tracking.
2. **controller**
This package contains the controllers to command the drone offboard.
3. **crazyflie_ros**
This is the package providing the services to communicate with the *Crazyflie* drone.
4. **demo_launchers**
This packages contains the *ROS* launch files for starting the system components.
Currently, there are
    1. "cf\_server.launch": Starts the nodes to communicate with the *Crazyflie*.
    2. "commander_launch": Starts the Commander node and the Guidance node.
    3. "controller_launch": Start the remote Controller node.
    4. "datastream_lauch": Starts the acquisition of data from the input sources, such as *Optitrack*. The launch file starts also an instance of *rViz* to visualize the vehicle/trajectory/estimation in a virtual environment.
    5. "demo_core": Start the datastream and the commander parts of the framework.
    6. "ext_control": Starts the offboard controller node.
    7. "test_launch": ...
5. **guidance**
This package provides the guidance for the drone, that is, given a command and the current status of the drone, it generates the references to achieve the task. 
6. **monitors**
This package contains nodes to convert *ROS* topics into a form that can be represented in *rViz*, such as odometry.
7. **state_aggregator**
This package provides the node that is aggregating the external information about the drone (currently, just the Optitrack) and provides an estimate of the drone state.
8. **testbed_msgs**
This package contains the common custom messages used by the nodes.


## Setting up the framwork
The project was developed in *ROS Melodic* and some modules used *Eigen*. Make sure to have *ROS* and Eigen available on your system. 

Dependencies:
- [Eigen](https://eigen.tuxfamily.org) -- a header-only linear algebra library for C++
- [python-scipy] -- python library for math computation

The repository contains submodules, thus, make sure you clone *recursively*.
To build the framework, navigate to the `ws/` directory (the _workspace_), and run
```
catkin_make
```

Every time you open a new terminal, you'll have to reload the references to the framework packages. Do this by running the following command from the `ws/` directory:
```
source devel/setup.bash
```

## Basic Example
Basic test:
It is possible to run a basic test to check the state\_aggregator, commander, guidance and visualization components.
The commands to launch the necessary modules are:
```
roslaunch demo_launchers datastream_launch.launch
```
```
roslaunch demo_launchers commander_launch.launch
```

These commands should start the rViz visualization tool and the basic nodes. 

If the Optitrack is connected to the machine and its VRPN server is streaming rigid body data (connection parameters should be configured in the "datastream\_launch.launch file) it should be possible to visualize the drone frame moving in the rViz environment.

It is possible to test the commander calling the ros service to request a goto movement.
The argument for the service is a tuple of float32, representing the position [x, y, z], and a float32 representing the duration of the requested movement. 
For example: 
```
rosservice call /cf1/Commander_Node/goTo_srv '[1.0, 1.0, 1.0]' '3.0'
```
The rViz visualizer should plot a reference frame (ghost) moving towards the requested point. That movement of that frame is generated simulating the requested trajectory. 
