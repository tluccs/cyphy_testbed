#!/usr/bin/env python

# This is the guidance node 
# It genereates the trajectory to reach a target point using 
# polynomials.
import numpy as np
import math

import rospy
import time
from threading import Thread
from tf.transformations import euler_from_matrix
from nav_msgs.msg import Odometry
from testbed_msgs.msg import ControlSetpoint 
from geometry_msgs.msg import PoseStamped

from guidance.srv import GenImpTrajectory
from guidance.srv import GenGoToTrajectory
from guidance.srv import GenImpTrajectoryAuto
from guidance.srv import GenTrackTrajectory

import trjgen.class_pwpoly as pw
import trjgen.class_trajectory as trj

current_odometry = Odometry()
current_target = PoseStamped()


############################   Helper functions

def quat2yaw(q):
    yaw = math.atan2(2.0 * (q[0] * q[3] + q[1] * q[2]),
            1.0 - 2.0 * (q[2]**2 + q[3]**2))
    return yaw

def quat2Z(q):
    Z = np.array([2.0 * (q[0]*q[2] + q[1]*q[3]),
         2.0*(q[2]*q[3] - q[0]*q[1]),
         1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])]) 
    return Z

def Mq(p):
    M = np.array([
        [p[0], -p[1], -p[2], -p[3]], 
        [p[1], p[0], -p[3], p[2]],
        [p[2], p[3], p[0], -p[1]],
        [p[3], -p[2], p[1], p[0]]
        ])
    return M

def quatMult(p, q):
    M = np.array([
        [p[0], -p[1], -p[2], -p[3]], 
        [p[1], p[0], -p[3], p[2]],
        [p[2], p[3], p[0], -p[1]],
        [p[3], -p[2], p[1], p[0]]
        ])
    res = np.matmul(M, q)


# Create the knots vector
def updateKnots(t_impact, Trec, dt):
    t1 = t_impact - dt
    t2 = t_impact + dt
    t3 = t_impact + dt + Trec
    
    # Check for queer situations
    if (t1 < 0.0):
        t1 = 0.0
        
    knots = np.array([0,
                      t1,
                      t_impact,
                      t2,
                      t3]
                    )
    return knots

def getInterpolMatrices(tg, tg_q, yaw, v_norm, a_norm, dt):
    
    # Extract the coordinates of the target Z axis from the rotation matrix
    # extressed with the quaternion
    tg_Zi = quat2Z(tg_q)

    a_dem = a_norm * tg_Zi - np.array([0.0, 0.0, 9.81])
    v_dem = - v_norm * tg_Zi

    v_aft = v_dem - a_dem * dt
    
    # Compute the waypoints near the target
    p_pre = tg - v_dem * dt 
    p_aft = tg + v_dem * dt  + 0.5 * (a_dem * dt**2)
    p_end = p_aft + v_aft * dt
    
    rospy.loginfo("Pre target = " +  str(p_pre))
    rospy.loginfo("Target = " +  str(tg))
    rospy.loginfo("Vel = " + str(v_dem))
    rospy.loginfo("Acc = " +  str(a_dem))
    rospy.loginfo("Post target pos = " + str(p_aft))
    rospy.loginfo("Post target vel = " + str(v_aft))
    rospy.loginfo("Recoil point = " + str(p_end))

    #   Relative waypoint data
    #   Start   PreTarget   Target      PostTarget  End
    X = np.array([
        [ 0,   p_pre[0],    tg[0],   p_aft[0],   p_end[0]],
        [ 0,   np.nan,      v_dem[0],   np.nan,     0.0],
        [ 0,   np.nan,      a_dem[0],   np.nan,     0.0],
        [ 0,   np.nan,      np.nan,     np.nan,     0.0]
    ])

    Y = np.array([
        [ 0,   p_pre[1],    tg[1],  p_aft[1],   p_end[1]],
        [ 0,   np.nan,      v_dem[1],   np.nan,     0.0],
        [ 0,   np.nan,      a_dem[1],   np.nan,     0.0],
        [ 0,   np.nan,      np.nan,     np.nan,     0.0]
    ])

    Z = np.array([
        [ 0,   p_pre[2],    tg[2],  p_aft[2],   p_end[2]],
        [ 0,   np.nan,      v_dem[2],   np.nan,     0.0],
        [ 0,   np.nan,      a_dem[2],   np.nan,     0.0],
        [ 0,   np.nan,      np.nan,     np.nan,     0.0]
    ])

    W = np.array([
        [ 0,   np.nan,   yaw,    np.nan,     0.0],
        [ 0,   np.nan,   np.nan,    np.nan,     0.0],
        [ 0,   np.nan,   np.nan,    np.nan,     0.0],
        [ 0,   np.nan,   np.nan,    np.nan,     0.0]
    ])
    
    return (X, Y, Z, W)




def odom_callback(odometry_msg):
    global current_odometry 
    current_odometry = odometry_msg
    return

def tg_callback(pose_msg):
    global current_target
    current_target = pose_msg
    return


def handle_genImpTrjAuto(req):
    
    ndeg = 7
    a_norm = req.a_norm
    v_norm = req.v_norm

    start_pos = np.array([current_odometry.pose.pose.position.x, 
            current_odometry.pose.pose.position.y, 
            current_odometry.pose.pose.position.z])
    start_orientation = np.array([current_odometry.pose.pose.orientation.w,
        current_odometry.pose.pose.orientation.x, 
        current_odometry.pose.pose.orientation.y,
        current_odometry.pose.pose.orientation.z]
    )

    tg_pos = np.array([current_target.pose.position.x,
            current_target.pose.position.y,
            current_target.pose.position.z])

    tg_q = np.array([current_target.pose.orientation.w,
        current_target.pose.orientation.x,
        current_target.pose.orientation.y,
        current_target.pose.orientation.z]
        )
   
    start_yaw = quat2yaw(start_orientation)
    tg_yaw = quat2yaw(tg_q)

    rospy.loginfo("On Target in " + str(req.t2go) + " sec!")
    rospy.loginfo("Target = [" + str(tg_pos[0]) + " " +
            str(tg_pos[1]) + " " + str(tg_pos[2]) + "]")
    rospy.loginfo("Vehicle = [" + str(start_pos[0]) + " " +
            str(start_pos[1]) + " " + str(start_pos[2]) + "]")

    # Generate the knots vector
    t_impact = req.t2go
    Trec = 1.0
    dt = 0.7
    knots = updateKnots(t_impact, Trec, dt)
    # Generate the interpolation matrices
    (X, Y, Z, W) = getInterpolMatrices(tg_pos - start_pos, tg_q, tg_yaw - start_yaw, v_norm, a_norm, dt)

    # Generate the polynomial
    ppx = pw.PwPoly(X, knots, ndeg)
    ppy = pw.PwPoly(Y, knots, ndeg)
    ppz = pw.PwPoly(Z, knots, ndeg)
    ppw = pw.PwPoly(W, knots, ndeg)

    frequency = rospy.get_param('~freq', 30.0);

    my_traj = trj.Trajectory(ppx, ppy, ppz, ppw)

    Dt = knots[1:len(knots)] - knots[0:len(knots)-1]

    my_traj.writeTofile(Dt, '/tmp/toTarget.csv')

    t = Thread(target=rep_trajectory, args=(my_traj, start_pos, max(knots), frequency)).start()
    return True



def handle_genImpTrj(req):
    start_pos = req.start_position
 
    tg = req.target
    t_impact = req.tg_time
     
    Tmax = 3 * t_impact;
    thrust_thr = 9.81 * vehicle_mass * 2.0

    # Times (Absolute and intervals)
    knots = np.array([0, t_impact, Tmax]) # One second each piece
    Dt = knots[1:len(knots)] - knots[0:len(knots)-1]

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

    my_traj.writeTofile(Dt, '/tmp/toTarget.csv')

    t = Thread(target=rep_trajectory, args=(my_traj,start_pos, Tmax, frequency)).start()
    return True


# Generate a tracking trajectory to reach an absolute waypoint 
# with a given velocity and acceleration.
def handle_genTrackTrj(req):
 
    start_pos = np.array([current_odometry.pose.pose.position.x, 
            current_odometry.pose.pose.position.y, 
            current_odometry.pose.pose.position.z])

    start_vel = np.array([current_odometry.twist.twist.linear.x, 
            current_odometry.twist.twist.linear.y, 
            current_odometry.twist.twist.linear.z])

    t_impact = req.tg_time
    
    tg_v = req.target_v
    tg_a = req.target_a


    # A little logic considering what could have been requested
    if (req.ref == "Absolute"):
        tg_p = req.target_p
        tg_prel = tg_p - start_pos
    else:
        tg_prel = req.target_p
        # Detect if you are requesting a landing
        if ((start_pos[2] + tg_prel[2]) <= 0):
            # I have to redefine because I cannot mutate tuples
            tg_prel = [tg_prel[0], tg_prel[1], -start_pos[2]]
            # Automatically calculate the time to land, if not specified
            if (t_impact == 0.0):
                t_impact = (start_pos[2]/0.3)
            tg_v = np.zeros((3))
            tg_a = np.zeros((3))
    
    # Times (Absolute and intervals)
    knots = np.array([0, t_impact]) # One second each piece
    Dt = knots[1:len(knots)] - knots[0:len(knots)-1]

    # Polynomial characteristic:  order
    ndeg = 7
    nconstr = 4

    X = np.array([
        [ 0.0,          tg_prel[0]],
        [ start_vel[0], tg_v[0]],
        [ 0.0,          tg_a[0]],
        [ 0.0,          0.0],
        ])

    Y = np.array([
        [ 0.0,          tg_prel[1]],
        [ start_vel[1], tg_v[1]],
        [ 0.0,          tg_a[1]],
        [ 0.0,          0.0],
        ])
    
    Z = np.array([
        [ 0.0,          tg_prel[2]],
        [ start_vel[2], tg_v[2]],
        [ 0.0,          tg_a[2]],
        [ 0.0,          0.0],
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

    my_traj.writeTofile(Dt, '/tmp/toTarget.csv')

    t = Thread(target=rep_trajectory, 
            args=(my_traj, start_pos, t_impact, frequency)).start()

    return True 


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

    return True 


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

    service_imp = rospy.Service('gen_ImpTrajectory', 
            GenImpTrajectory, handle_genImpTrj)

    service_track = rospy.Service('gen_TrackTrajectory', 
            GenTrackTrajectory, handle_genTrackTrj)

    service_imp_auto = rospy.Service('gen_ImpTrajectoryAuto', 
            GenImpTrajectoryAuto, handle_genImpTrjAuto)

    # Subscribe to vehicle state update
    rospy.Subscriber(dr_odom_topic_, Odometry, odom_callback)
    rospy.Subscriber(tg_pose_topic_, PoseStamped, tg_callback)

    # Setpoint Publisher
    ctrl_setpoint_pub = rospy.Publisher(ctrlsetpoint_topic_, 
            ControlSetpoint, queue_size=10)

    rospy.spin()

