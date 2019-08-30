#!/bin/sh

printf "Requesting: %s\n" $1

if [ $1 = "land" ] || [ $1 = "Land" ] || [ $1 = "l" ]; then
	rosservice call /cf1/Commander_Node/land_srv "duration: 2.0"
	sleep 2 
	rosservice call /cf1/stop "groupMask: 0"
fi

if [ $1 = "takeoff" ] || [ $1 = "Takeoff" ] || [ $1 = "t" ]; then
	rosservice call /cf1/Commander_Node/takeoff_srv '1.2' '3.0'
fi

if [ $1 = "stop" ] || [ $1 = "Stop" ] || [ $1 = "s" ]; then
	rosservice call /cf1/stop "groupMask: 0"
fi

if [ $1 = "auto" ] || [ $1 = "Auto" ] || [ $1 = "a" ]; then
	rosservice call /cf1/gen_ImpTrajectoryAuto '0.5' '9' '1.6'
fi

if [ $1 = "waypoints" ] || [ $1 = "Waypoints" ] || [ $1 = "w" ]; then
	# Take off
	rosservice call /cf1/Commander_Node/takeoff_srv '0.5' '3.0'
	sleep 3
	# Waypoints
	rosservice call /cf1/Commander_Node/goTo_srv '[1.0, 1.0, 1.0]' '4.0'
	sleep 4
	rosservice call /cf1/Commander_Node/goTo_srv '[-1.3, 1.0, 1.0]' '2.0'
	sleep 2
	rosservice call /cf1/Commander_Node/goTo_srv '[-1.3, -1.0, 1.0]' '3.0'
	sleep 3
	rosservice call /cf1/Commander_Node/goTo_srv '[-1.3, 1.0, 1.0]' '3.0'
	sleep 3	
	# Go to the table
	rosservice call /cf1/Commander_Node/goTo_srv '[-0.96, 1.56, 1.0]' '2.0'
	sleep 2 
	rosservice call /cf1/Commander_Node/goTo_srv '[-0.96, 1.56, 0.8]' '2.0'
	sleep 2
	
	# Stop
	rosservice call /cf1/stop "groupMask: 0"
	rosservice call /cf1/stop "groupMask: 0"
	#rosservice call /cf1/Commander_Node/goTo_srv '[0.0, 0.0, 1.0]' '2.0'
	#sleep 2
	#rosservice call /cf1/Commander_Node/land_srv "duration: 2.0"
fi

return 0

