import numpy as np
import math
import time



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
def updateKnots(t_impact, dt, Trec):
    t1 = t_impact - dt
    t2 = t_impact + Trec
        
    knots = np.array([0,
                      t1,
                      t_impact,
                      t2]
                    )
    return knots


def AddConstraint(A, constr):
    N = constr.size
    col_constr = constr.reshape(N,1)
    if (A.size == 0):
        A = col_constr
    else:
        A = np.hstack((A, col_constr))
    return A

# Integration Step 
def integrationStep(X, u, dt, direction):
    # x(k+1) = A(dt) x(k)  + B(dt) u(k)
    A = np.eye((6), dtype=float)
    A[0][3] = dt
    A[1][4] = dt
    A[2][5] = dt

    B1 = np.eye((3), dtype=float) * (dt**2)/2
    B2 = np.eye((3), dtype=float) * dt
    B = np.vstack((B1, B2))
    
    if (direction == -1):
        # x(k) = A_(dt) x(k+1) + B_(dt) u(k)
        A = np.linalg.inv(A)
        B = -1 * np.matmul(A, B)
  
  #  print("A")
  #  print(A)

  #  print("\nB")
  #  print(B)

  #  print("\nX")
  #  print(X)

  #  print("\nu")
  #  print(u)
    X = np.matmul(A, X) + np.matmul(B, u)
    return X


# Back integration
def Integration(p, v, a, Tf, dt, direction):  
    t = 0.0

    # State vector composed by position and velocity
    p = np.resize(p, (3,1))
    v = np.resize(v, (3,1))
    a = np.resize(a, (3,1))
    X = np.vstack((p, v))

    while (t < Tf):
        X = integrationStep(X, a, dt, direction)
        t = t + dt

    return X



# Generate the matrices for the interpolation problem
def genInterpolProblem(tg, tg_q, yaw, v_norm, a_norm):
    
    # Extract the coordinates of the target Z axis from the rotation matrix
    # extressed with the quaternion
    tg_Zi = np.array([2.0*tg_q[0]*tg_q[2] + 2.0*tg_q[1]*tg_q[3],
         2.0*(tg_q[2]*tg_q[3] - tg_q[0]*tg_q[1]),
         tg_q[0]*tg_q[0] -tg_q[1]*tg_q[1] -tg_q[2]*tg_q[2] + tg_q[3]*tg_q[3]]) 

    a_dem = a_norm * tg_Zi - np.array([0.0, 0.0, 9.81])
    v_dem = -v_norm * tg_Zi
   
    DT = 0.2
    Trec = 4 * DT

    # Compute the waypoints near the target
    # I am considering moving with a constant acceleration (negative), while going towards the target
    xv_pre = Integration(tg, v_dem, a_dem, DT, 0.001, -1) 
    p_pre = np.reshape(xv_pre[0:3], (3,))
    v_pre = np.reshape(xv_pre[3:6], (3,))
   
    xv_post = Integration(tg, v_dem, a_dem, DT, 0.001, 1) 
    p_post = np.reshape(xv_post[0:3], (3,))
    v_post = np.reshape(xv_post[3:6], (3,))

    rec_v = np.copy(v_post)
    rec_v[2] = 0.0;
    p_end = p_post + rec_v * Trec 

    print("Relative Target = \n", tg)
    print("Vel = \n", v_dem)
    print("Acc = \n", a_dem)

    print("Pre target point = ")
    print(p_pre)
    print("Pre target velocity = ")
    print(v_pre)
    
    print("Post target point = ")
    print(p_post)
    print("Post target velocity = ")
    print(v_post)

    print("Recoil point = \n ", p_end)
    
    X = np.array([[]])
    Y = np.array([[]])
    Z = np.array([[]])
    W = np.array([[]])
    
    
    xpre = np.array([p_pre[0], v_pre[0], a_dem[0]])
    xtrg = np.array([tg[0], v_dem[0], a_dem[0]])
    xpost = np.array([p_post[0], v_post[0], np.nan])
    xend = np.array([p_end[0], 0, 0])
    
    ypre = np.array([p_pre[1], v_pre[1], a_dem[1]])
    ytrg = np.array([tg[1], v_dem[1], a_dem[1]])
    ypost = np.array([p_post[1], v_post[1], np.nan])
    yend = np.array([p_end[1], 0, 0])

    zpre = np.array([p_pre[2], v_pre[2], a_dem[2]])
    ztrg = np.array([tg[2], v_dem[2], a_dem[2]])
    zpost = np.array([p_post[2], v_post[2], np.nan])
    zend = np.array([p_end[2], 0, 0])
    
    X = AddConstraint(X, np.array([0,0,0]))
    X = AddConstraint(X, xpre)
    X = AddConstraint(X, xtrg)
    X = AddConstraint(X, xpost)
    X = AddConstraint(X, xend)
    
    Y = AddConstraint(Y, np.array([0,0,0]))
    Y = AddConstraint(Y, ypre)
    Y = AddConstraint(Y, ytrg)
    Y = AddConstraint(Y, ypost)
    Y = AddConstraint(Y, yend)
    
    Z = AddConstraint(Z, np.array([0,0,0]))
    Z = AddConstraint(Z, zpre)
    Z = AddConstraint(Z, ztrg)
    Z = AddConstraint(Z, zpost)
    Z = AddConstraint(Z, zend)
    

    W = AddConstraint(W, np.array([0,0,0]))
    W = AddConstraint(W, np.array([np.nan, np.nan, np.nan]))
    W = AddConstraint(W, np.array([yaw, np.nan, np.nan]))
    W = AddConstraint(W, np.array([np.nan, np.nan, np.nan]))
    W = AddConstraint(W, np.array([0,0,0]))
   
    # Generate the relative array of knots with respect
    # to the impact time (0.0)
    relknots = np.array([-DT, 0.0, DT, Trec])
   
    return (X, Y, Z, W, relknots)


def getInterpolMatrices(tg, tg_q, yaw, v_norm, a_norm, dt):
    
    # Extract the coordinates of the target Z axis from the rotation matrix
    # extressed with the quaternion
    tg_Zi = quat2Z(tg_q)

    a_dem = a_norm * tg_Zi - np.array([0.0, 0.0, 9.81])
    v_dem = - v_norm * tg_Zi

    # Compute the waypoints near the target
    p_end = tg + v_dem * dt
    
    rospy.loginfo("Rel Target = " +  str(tg))
    rospy.loginfo("Vel = " + str(v_dem))
    rospy.loginfo("Acc = " +  str(a_dem))
    rospy.loginfo("Rel Recoil point = " + str(p_end))

    #   Relative waypoint data
    #   Start   PreTarget   Target      PostTarget  End
    X = np.array([
        [ 0,   tg[0],      p_end[0]],
        [ 0,   v_dem[0],   0.0],
        [ 0,   a_dem[0],   0.0],
        [ 0,   np.nan,     0.0]
    ])

    Y = np.array([
        [ 0,   tg[1],      p_end[1]],
        [ 0,   v_dem[1],   0.0],
        [ 0,   a_dem[1],   0.0],
        [ 0,   np.nan,     0.0]
    ])

    Z = np.array([
        [ 0,   tg[2],      p_end[2]],
        [ 0,   v_dem[2],   0.0],
        [ 0,   a_dem[2],   0.0],
        [ 0,   np.nan,     0.0]
    ])

    W = np.array([
        [ 0,   yaw,       0.0],
        [ 0,   np.nan,    0.0],
        [ 0,   np.nan,    0.0],
        [ 0,   np.nan,    0.0]
    ])
    
    return (X, Y, Z, W)



