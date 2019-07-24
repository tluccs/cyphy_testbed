#!/usr/bin/env python

# This is a one shot planner node.
# It genereates the trajectory to reach a target point using 
# polynomials.
import numpy as np

import rospy
import time
from threading import Thread
from tf.transformations import euler_from_matrix
from testbed_msgs.msg import Setpoint 
from testbed_srvs.srv import GenImpTrajectory, GenGoToTrajectory

import trjgen.class_pwpoly as pw
import trjgen.class_trajectory as trj

# Setpoint Publisher
ghost_pub = rospy.Publisher('setpoint', Setpoint, queue_size=10)

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


def handle_genGotoTrj():
    tg_p = req.target_p
    tg_v = req.target_v
    tg_a = req.target_a
    t_impact = req.tg_time
     
    # Times (Absolute and intervals)
    knots = np.array([0, t_impact]) # One second each piece

    # Polynomial characteristic:  order
    ndeg = 7
    nconstr = 4

    rospy.loginfo("On Target in " + str(t_impact) + " sec!")
    rospy.loginfo("Target = [" + str(tg_x) + " " + str(tg_y) + 
            " " + str(tg_z) + "]")

    X = np.array([
        [ 0.0,    tg_p[0]],
        [ 0.0,    tg_v[0]],
        [ 0.0,    tg_a[0]],
        [ 0.0,    np.nan],
        ])

    Y = np.array([
        [ 0.0,    tg_p[1]],
        [ 0.0,    tg_v[1]],
        [ 0.0,    tg_a[1]],
        [ 0.0,    np.nan],
        ])
    
    Z = np.array([
        [ 0.0,    tg_p[2]],
        [ 0.0,    tg_v[2]],
        [ 0.0,    tg_a[2]],
        [ 0.0,    np.nan],
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

    start_pos = [0.0, 0.0, 0.0]
    rep_trajectory(my_traj, start_pos, Tmax, frequency)


def rep_trajectory(traj, start_position, timeSpan, freq):
    r = rospy.Rate(freq)

    start_time = rospy.get_time() 
    curr_time = start_time
    end_time = start_time + timeSpan

    msg = Setpoint()

    # Publishing Loop
    while (curr_time < end_time):
        # Evaluate the trajectory
        (X, Y, Z, W, R) = traj.eval(curr_time - start_time, [0, 1, 2])

        msg.px = X[0] 
        msg.py = Y[0] 
        msg.pz = Z[0] 

        msg.vx = X[1] 
        msg.vy = Y[1]
        msg.vz = Z[1]

        msg.accx = X[2]
        msg.accy = Y[2]
        msg.accz = Z[2]

        # Conver the Rotation matrix to euler angles
        (roll, pitch, yaw) = euler_from_matrix(R)

        msg.r = roll
        msg.p = pitch
        msg.y = yaw

        # Pubblish the evaluated trajectory
        ghost_pub.publish(msg)

        # Wait the next loop
        r.sleep()
        # Take the time
        curr_time = rospy.get_time()


if __name__ == '__main__':
    rospy.init_node('Trajectory_Generator_Node')

    vehicle_mass = rospy.get_param('~vehicle_mass', 0.032)
    n_points = rospy.get_param('traject_points', 100) 

    s = rospy.Service('gen_ImpTrajectory', GenImpTrajectory, handle_genImpTrj)
    s = rospy.Service('gen_goToTrajectory', GenGoToTrajectory, handle_genGotoTrj) 

    rospy.spin()

