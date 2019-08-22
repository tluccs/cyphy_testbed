#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Helper functions for trajectory generation and plotting


@author: @author: l.pannocchi@gmail.com
"""
import numpy as np
import numpy.polynomial.polynomial as pl
from mpl_toolkits.mplot3d import Axes3D

import matplotlib
from matplotlib import pyplot as plt

matplotlib.use('qt5agg')

## =================================================
## =================================================
def TrajFromPW(Tv, derlist, pwpolx=None, pwpoly=None, pwpolz=None, pwpolw=None):
    """
    Generate the trjactory at the time Tv from the polynomial describing the
    X, Y, Z and Y.

    Args
        Tv: Time vector
        derlist: List of the order of derivatives to be evaluated
        pwpolx: Polynomial describing the position on the X
        pwpoly: Polynomial describing the position on the Y
        pwpolz: Polynomial describing the position on the Z
        pwpolw: Polynomial describing the position on the Yaw

    Output
        X,Y,Z,W: Flat output trajectory
        Zb: Vector along the Z body axis

    """
    G = 9.81

    if (pwpolx != None):
        X = pwpTo1D(Tv, pwpolx, derlist)
    else:
        X = np.zeros((len(derlist), len(Tv)))

    if (pwpoly != None):
        Y = pwpTo1D(Tv, pwpoly, derlist)
    else:
        Y = np.zeros((len(derlist), len(Tv)))

    if (pwpolz != None):
        Z = pwpTo1D(Tv, pwpolz, derlist)
    else:
        Z = np.zeros((len(derlist), len(Tv)))

    if (pwpolw != None):
        W = pwpTo1D(Tv, pwpolw, derlist)
    else:
        W = np.zeros((len(derlist), len(Tv)))

    if (np.array(derlist) == 2).any():
        ACC = np.concatenate(([X[2,:]], [Y[2,:]], [Z[2,:] + G]), axis = 0)
        Zb = (ACC / np.linalg.norm(ACC, axis = 0))

    return (X, Y, Z, W, Zb)


## =================================================
## =================================================
def plotTraj(X, Y, Z, W, Zb, Tv, derlist, scaleZb=0.01):
    """
    Plot the flat output trajectory as a scatter plot. The color of
    the plot is mapped on the time.
    A quiver plot is superimposed to show the direction of the requested
    thrust vector during the trajectory.

    Args
        X,Y,Z,W:    Flat output trajectory
        Zb:         Vector along the Z body axis
        Tv:         Time vector
        derlist:    List of the order of derivatives to be evaluated
        scaleZb:    Scaling factor to improve the visualization of the Zb vector
                    direction

    Output

    """

    for i in range(len(derlist)):
        # Create the figure and axes
        plt.figure()
        ax = plt.axes(projection='3d')
        title_str = 'Derivative n = {:1d}'.format(derlist[i])
        print(title_str)
        ax.set_title(title_str)
        if derlist[i] == 0:
            ax.set_xlabel("x [m]")
            ax.set_ylabel("y [m]")
            ax.set_zlabel("z [m]")
        if derlist[i] == 1:
            ax.set_xlabel("x [m/s]")
            ax.set_ylabel("y [m/s]")
            ax.set_zlabel("z [m/s]")
        if derlist[i] == 2:
            ax.set_xlabel("x [m/s^2]")
            ax.set_ylabel("y [m/s^2]")
            ax.set_zlabel("z [m/s^2]")

        p = ax.scatter(X[i, :], Y[i, :], Z[i, :], c = Tv)
        plt.colorbar(p)
        Zb = Zb * scaleZb
        if (derlist[i] == 0):
            ax.quiver(X[i, :], Y[i, :], Z[i, :], Zb[0, :], Zb[1, :],\
                    Zb[2,:])
       
        plt.show()


## =================================================
## =================================================
def plotPoly3D(Dt, Nsamples, polx, poly, polz, derlist):
    """
    Plot the trajectory coordinates as a scatter plot
    """

    (T, X, Y, Z) = polysTo3D(Dt, Nsamples, polysx=polx, \
            polysy=poly, polysz=polz, der=derlist)

    for i in range(len(derlist)):
        # Create the figure and axes
        plt.figure(i)
        ax = plt.axes(projection='3d')
        title_str = 'Derivative n = {:1d}'.format(derlist[i])
        print(title_str)
        ax.set_title(title_str)
        if derlist[i] == 0:
            ax.set_xlabel("x [m]")
            ax.set_ylabel("y [m]")
            ax.set_zlabel("z [m]")
        if derlist[i] == 1:
            ax.set_xlabel("x [m/s]")
            ax.set_ylabel("y [m/s]")
            ax.set_zlabel("z [m/s]")
        if derlist[i] == 2:
            ax.set_xlabel("x [m/s^2]")
            ax.set_ylabel("y [m/s^2]")
            ax.set_zlabel("z [m/s^2]")

        ax.scatter(X[i, :], Y[i, :], Z[i, :])
        plt.gca().set_aspect('equal', adjustable='box')
        plt.show()

def plotThrustMargin(T, X, Y, Z, vehicle_mass, thrust_constr):
    """
    Plot the trajectory coordinates as a scatter plot evaluating also
    the thrust margin available for maneuvers.
    """

    (ffthrust, available_thrust) = getlimits(X, Y, Z, vehicle_mass, thrust_constr)

    perc_available = (available_thrust / thrust_constr) * 100.0
    if ((X.shape[1] < 3) or (Y.shape[1] < 3) or (Z.shape[1] < 3)):
        print("The trajectory should contain at least acceleration")
        return 0
    for i in range(3):
        # Create the figure and axes
        plt.figure()
        ax = plt.axes(projection='3d')
        title_str = 'Thrust Marging [Derivative n = {:1d}]'.format(i)
        print(title_str)
        ax.set_title(title_str)
        if i == 0:
            ax.set_xlabel("x [m]")
            ax.set_ylabel("y [m]")
            ax.set_zlabel("z [m]")
        if i == 1:
            ax.set_xlabel("x [m/s]")
            ax.set_ylabel("y [m/s]")
            ax.set_zlabel("z [m/s]")
        if i == 2:
            ax.set_xlabel("x [m/s^2]")
            ax.set_ylabel("y [m/s^2]")
            ax.set_zlabel("z [m/s^2]")

        p = ax.scatter(X[i, :], Y[i, :], Z[i, :], c=perc_available)
        plt.colorbar(p)

    plt.figure()
    plt.plot(T, ffthrust)
    plt.plot(T, np.repeat(thrust_constr, ffthrust.size))
    plt.plot(T, available_thrust)
    plt.xlabel("Time [s]")
    plt.ylabel("Thrust [N]")
    plt.legend(("FF Thrust", "Thrust Constraint", "Available Margin"))
    plt.grid(True)
    plt.show()

    return (ffthrust, available_thrust)


######################################################################
## Helper functions for polynomial plotting

def jointPieces(Dt, Nsamples, polys, der_list):
    # Number of derivates to evaluate
    numDer = len(der_list)

    # Number of pieces to join
    numPieces = len(Dt)

    # Time offsets
    toff = np.zeros(numPieces + 1);
    toff[1: numPieces + 1] = \
            np.matmul(np.tril(np.ones((numPieces, numPieces)), 0), np.array(Dt))

    # Matrix of time and output vector to represent the pieces
    t = np.zeros((numPieces, Nsamples))
    y = np.zeros((numPieces, Nsamples))

    # The overall time vector to include all the pieces
    T = np.zeros((numPieces * Nsamples))
    Y = np.zeros((numDer, numPieces * Nsamples))

    # Evaluation
    for i in range(len(Dt)):
        dt = Dt[i]/Nsamples
        t[i, :] = np.arange(0, Nsamples) * dt
        for j in range(numDer):
            # Evaluate the polynomial
            pder = pl.polyder(polys[i, :], der_list[j])
            y[i,:] = pl.polyval(t[i, :], pder)
            # Compose the single vectors
            Y[j, i * Nsamples: (i + 1) * Nsamples] = y[i, :]
            T[i * Nsamples: (i + 1) * Nsamples] = t[i, :] + toff[i]

    return (T, Y)


## =================================================
## =================================================
def polysTo1D(Dt, Nsamples, polysx, der):
    """
    Generate the 1D trajectory in space from polynomials
    """
    [T, X] = jointPieces(Dt, Nsamples, polysx, der)

    return (T, X)


## =================================================
## =================================================
def polysTo3D(Dt, Nsamples, polysx, polysy, polysz, der):
    """
    Generate the 3D trajectory in space from polynomials
    """
    [T, X] = jointPieces(Dt, Nsamples, polysx, der)
    [_, Y] = jointPieces(Dt, Nsamples, polysy, der)
    [_, Z] = jointPieces(Dt, Nsamples, polysz, der)

    return (T, X, Y, Z)



def getlimits(X, Y, Z, vehicle_mass, thrust_constr):
    """
    Function that computes the thrust marging for a given trajectory

    """

    grav_acc = 9.81

    # Check whether I have the acceleration data
    if ((X.shape[0] < 3) or (Y.shape[0] < 3) or (Z.shape[0] < 3)):
        print("The trajectory should contain at least acceleration")
        return 0

    if (X.ndim > 1):
        acc_x = np.expand_dims(X[2, :], axis=0)
        acc_y = np.expand_dims(Y[2, :], axis=0)
        acc_z = np.expand_dims(Z[2, :] + grav_acc, axis=0)
        acc_v = np.concatenate((acc_x, acc_y, acc_z), axis = 0)
    else:
        acc_x = X[2]
        acc_y = Y[2]
        acc_z = Z[2]
        acc_v = np.vstack((acc_x, acc_y, acc_z))


    ffthrust = vehicle_mass * (np.linalg.norm(acc_v, axis = 0, ord=2))
    available_thrust = thrust_constr - ffthrust

    return (ffthrust, available_thrust)


## =================================================
## =================================================
def pwpTo1D(T, pwpoly, der):
    """
    Generate the 1D trajectory in space from polynomial

    Args
        T:      Vector of points where the polynomial is evaluated
        pwpoly: Piecewise polynomial object
        der:    Order of the derivatives to be evaluated

    Outputs
        X:      Vector/Matrix with the evaluated polynomial
                NumDerivative x NumberOfPoints
    """

    # Check whether the input is a scalar
    if type(der) == int:
        der = [der]

    der = np.array(der)
    T = np.array(T)

    X = np.zeros((der.size, T.size))

    for i in range(der.size):
        X[i,:] = pwpoly.eval(T, der[i])

    return X
