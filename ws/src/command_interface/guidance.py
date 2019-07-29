#!/usr/bin/env python

# This is the guidance node 
# It genereates the trajectory to reach a target point using 
# polynomials.
import numpy as np

import rospy
import time
from threading import Thread
from tf.transformations import euler_from_matrix
from nav_msgs.msg import Odometry
from testbed_msgs.msg import ControlSetpoint 
from testbed_srvs.srv import GenImpTrajectory, GenGoToTrajectory, GenImpTrajectoryAuto
from geometry_msgs.msg import PoseStamped

import trjgen.class_pwpoly as pw
import trjgen.class_trajectory as trj

current_odometry = Odometry()
current_target = PoseStamped()

def odom_callback(odometry_msg):
    global current_odometry 
    current_odometry = odometry_msg
    return

def tg_callback(pose_msg):
    global current_target
    current_target = pose_msg
    return


def handle_genImpTrjAuto(req):
    start_pos = np.array([current_odometry.pose.pose.position.x, 
            current_odometry.pose.pose.position.y, 
            current_odometry.pose.pose.position.z])

    tg = np.array([current_target.pose.position.x, 
            current_target.pose.position.y, 
            current_target.pose.position.z])

    tg_rel = tg - start_pos

    t_impact = req.t2go
     
    Tmax = 3 * t_impact; 
    thrust_thr = 9.81 * vehicle_mass * 2.0

    # Times (Absolute and intervals)
    knots = np.array([0, t_impact, Tmax]) # One second each piece

    # Polynomial characteristic:  order
    ndeg = 7
    nconstr = 4

    # Speed at the target
    Vt = 4.0
    # Acceleration at the target
    At = 9.0

    tg_x = tg_rel[0]
    tg_y = tg_rel[1]
    tg_z = tg_rel[2]

    rospy.loginfo("On Target in " + str(t_impact) + " sec!")
    rospy.loginfo("Vehicle = [" + str(start_pos[0]) + " " + str(start_pos[1]) + 
            " " + str(start_pos[2]) + "]")

    rospy.loginfo("Target Abs = [" + str(tg[0]) + " " + str(tg[1]) + 
            " " + str(tg[2]) + "]")

    rospy.loginfo("Target = [" + str(tg_x) + " " + str(tg_y) + 
            " " + str(tg_z) + "]")

    X = np.array([
        [ 0,   tg_x,    tg_x + 0.10, 2.5 * tg_x],
        [ 0,   -Vt * tg_x/abs(tg_x), np.nan,   0.0],
        [ 0,   -At * tg_x/abs(tg_x),  np.nan ,  0.0],
        [ 0,   np.nan,  np.nan, 0.0],
        ])

    Y = np.array([
        [ 0,   tg_y,   np.nan, tg_y],
        [ 0,   0.0,    np.nan, 0.0],
        [ 0,   0.0,    np.nan, 0.0],
        [ 0,   np.nan, np.nan, 0.0],
        ])

    Z = np.array([
        [ 0,   tg_z,   np.nan,  0.0],
        [ 0,   0.0,    np.nan,  0.0],
        [ 0,   0.0,    np.nan,  0.0],
        [ 0,   np.nan, np.nan,  0.0],
        ])

    W = np.array([
        [ 0,   np.nan,   np.nan,   0.0],
        [ 0,   np.nan,   np.nan,   0.0],
        [ 0,   np.nan,   np.nan,   0.0],
        [ 0,   np.nan,   np.nan,   0.0],
        ])    


    ppx = pw.PwPoly(X, knots, ndeg)
    ppy = pw.PwPoly(Y, knots, ndeg)
    ppz = pw.PwPoly(Z, knots, ndeg)
    ppw = pw.PwPoly(W, knots, ndeg)

    frequency = rospy.get_param('~freq', 30.0);

    my_traj = trj.Trajectory(ppx, ppy, ppz, ppw)

    knots = np.array([0, t_impact, t_impact + 0.5, Tmax])
    Dt = knots[1:len(knots)] - knots[0:len(knots)-1]

    my_traj.writeTofile(Dt, '/tmp/toTarget.csv')

    t = Thread(target=rep_trajectory, args=(my_traj, start_pos, Tmax, frequency)).start()
    return "Roger!"



def handle_genImpTrj(req):
    start_pos = req.start_position 
 
    tg = req.target 
    t_impact = req.tg_time
     
    Tmax = 3 * t_impact; 
    thrust_thr = 9.81 * vehicle_mass * 2.0

    # Times (Absolute and intervals)
    knots = np.array([0, t_impact, Tmax]) # One second each piece

    # Polynomial characteristic:  order
    ndeg = 7
    nconstr = 4

    # Speed at the target
    Vt = 4.0
    # Acceleration at the target
    At = 9.0

    tg_x = tg[0]
    tg_y = tg[1]
    tg_z = tg[2]

    rospy.loginfo("On Target in " + str(t_impact) + " sec!")
    rospy.loginfo("Target = [" + str(tg_x) + " " + str(tg_y) + 
            " " + str(tg_z) + "]")

    X = np.array([
        [ 0,     tg_x,    2.5 * tg_x],
        [ 0,     -Vt * tg_x/abs(tg_x),   0.0],
        [ 0,   -At * tg_x/abs(tg_x),     0.0],
        [ 0,   np.nan,   0.0],
        ])

    Y = np.array([
        [ 0,   tg_y,      0.0],
        [ 0,   0.0,      0.0],
        [ 0,   0.0,      0.0],
        [ 0,   np.nan,   0.0],
        ])

    Z = np.array([
        [ 0,   tg_z,      0.0],
        [ 0,   0.0,      0.0],
        [ 0,   0.0,      0.0],
        [ 0,   np.nan,   0.0],
        ])

    W = np.array([
        [ 0,   np.nan,      0.0],
        [ 0,   np.nan,      0.0],
        [ 0,   np.nan,      0.0],
        [ 0,   np.nan,      0.0],
        ])    


    ppx = pw.PwPoly(X, knots, ndeg)
    ppy = pw.PwPoly(Y, knots, ndeg)
    ppz = pw.PwPoly(Z, knots, ndeg)
    ppw = pw.PwPoly(W, knots, ndeg)

    frequency = rospy.get_param('~freq', 30.0);

    my_traj = trj.Trajectory(ppx, ppy, ppz, ppw)
 
    t = Thread(target=rep_trajectory, args=(my_traj,start_pos, Tmax, frequency)).start()
    return "Roger!"


def handle_genGotoTrj(req):
    tg_p = req.target_p
    tg_v = req.target_v
    tg_a = req.target_a
    t_impact = req.tg_time
     
    # Times (Absolute and intervals)
    knots = np.array([0, t_impact]) # One second each piece

    # Polynomial characteristic:  order
    ndeg = 7
    nconstr = 4

    X = np.array([
        [ 0.0,    tg_p[0]],
        [ 0.0,    tg_v[0]],
        [ 0.0,    tg_a[0]],
        [ 0.0,    0.0],
        ])

    Y = np.array([
        [ 0.0,    tg_p[1]],
        [ 0.0,    tg_v[1]],
        [ 0.0,    tg_a[1]],
        [ 0.0,    0.0],
        ])
    
    Z = np.array([
        [ 0.0,    tg_p[2]],
        [ 0.0,    tg_v[2]],
        [ 0.0,    tg_a[2]],
        [ 0.0,    0.0],
        ])

    W = np.array([
        [ 0.0,    0.0],
        [ 0.0,    0.0],
        [ 0.0,    0.0],
        [ 0.0,    0.0],
        ])

    ppx = pw.PwPoly(X, knots, ndeg)
    ppy = pw.PwPoly(Y, knots, ndeg)
    ppz = pw.PwPoly(Z, knots, ndeg)
    ppw = pw.PwPoly(W, knots, ndeg)

    frequency = rospy.get_param('~freq', 30.0);

    my_traj = trj.Trajectory(ppx, ppy, ppz, ppw)

    start_pos = [current_odometry.pose.pose.position.x, 
            current_odometry.pose.pose.position.y, 
            current_odometry.pose.pose.position.z]

    t = Thread(target=rep_trajectory, 
            args=(my_traj, start_pos, t_impact, frequency)).start()

    return "Roger!"


def rep_trajectory(traj, start_position, timeSpan, freq):
    global ctrl_setpoint_pub

    r = rospy.Rate(freq)

    start_time = rospy.get_time() 
    curr_time = start_time
    end_time = start_time + timeSpan

    msg = ControlSetpoint()
    rtime = rospy.get_rostime()
    msg.header.stamp = rtime

    # Publishing Loop
    while (curr_time < end_time):
        # Evaluate the trajectory
        (X, Y, Z, W, R) = traj.eval(curr_time - start_time, [0, 1, 2])

        msg.p.x = X[0] + start_position[0]
        msg.p.y = Y[0] + start_position[1]
        msg.p.z = Z[0] + start_position[2] 

        msg.v.x = X[1] 
        msg.v.y = Y[1]
        msg.v.z = Z[1]

        msg.a.x = X[2]
        msg.a.y = Y[2]
        msg.a.z = Z[2]

        # Conver the Rotation matrix to euler angles
        (roll, pitch, yaw) = euler_from_matrix(R)

        msg.rpy.x = roll
        msg.rpy.y = pitch
        msg.rpy.z = yaw
 
        # Pubblish the evaluated trajectory
        ctrl_setpoint_pub.publish(msg)

        # Wait the next loop
        r.sleep()
        # Take the time
        curr_time = rospy.get_time()


if __name__ == '__main__':
    rospy.init_node('Guidance_Node')

    vehicle_mass = rospy.get_param('~vehicle_mass', 0.032)
    n_points = rospy.get_param('traject_points', 100) 

    commander_id = rospy.get_param('~command_id', 'cm1')
    target_frame = rospy.get_param('~target_frame', 'cf1')
    
    dr_odom_topic_ = rospy.get_param('topics/in_vehicle_odom_topic', 'external_odom')
    tg_pose_topic_ = rospy.get_param('topics/in_tg_pose_topic', "/vrpn_client_node/target/pose")

    ctrlsetpoint_topic_ = rospy.get_param('topics/out_ctrl_setpoint', "setpoint")

    service_imp = rospy.Service('gen_ImpTrajectory', GenImpTrajectory, handle_genImpTrj)
    service_goto = rospy.Service('gen_goToTrajectory', GenGoToTrajectory, handle_genGotoTrj)

    service_imp_auto = rospy.Service('gen_ImpTrajectoryAuto', GenImpTrajectoryAuto, handle_genImpTrjAuto)

    # Subscribe to vehicle state update
    rospy.Subscriber(dr_odom_topic_, Odometry, odom_callback)
    rospy.Subscriber(tg_pose_topic_, PoseStamped, tg_callback)

    # Setpoint Publisher
    ctrl_setpoint_pub = rospy.Publisher(ctrlsetpoint_topic_, ControlSetpoint, queue_size=10)

    rospy.spin()

