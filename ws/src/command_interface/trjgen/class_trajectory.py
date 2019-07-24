#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: rt-2pm2
"""
import numpy as np

class Trajectory:
    """ 
    Class representing trajectories
    """

    ## Constructor
    def __init__(self, pwpx, pwpy, pwpz, pwpw):
        self.px = pwpx
        self.py = pwpy
        self.pz = pwpz
        self.pw = pwpw


    ## Evaluate the trajectory at a time 't' over a list
    ## of derivative 'derlist'
    def eval(self, t, derlist):
        # Check whether the input is a scalar
        if type(derlist) == int:
            der = [derlist]

        der = np.array(derlist)

        X = np.zeros((der.size))
        Y = np.zeros((der.size))
        Z = np.zeros((der.size))
        W = np.zeros((der.size))

        # Evaluate the polynomials on the requested
        # derivatives
        for i in range(der.size):
            X[i] = self.px.eval(t, der[i])
            Y[i] = self.py.eval(t, der[i])
            Z[i] = self.pz.eval(t, der[i])
            W[i] = self.pw.eval(t, der[i])

        # If the acceleration has been required, it's possible to compute
        # the demanded orientation of the vehicle

        R = np.zeros((3,3), dtype=float)
        if (np.array(derlist) == 2).any():

            # Assemble the acceleration 3d Vector
            Thrust = np.array((X[2], Y[2], Z[2] + 9.81))

            # Find the demanded Zb axis 
            Zb = (Thrust / np.linalg.norm(Thrust))

            X_w = np.array((np.cos(W[0]), np.sin(W[0])))
            Yb = np.cross(Zb, X_w)
            Yb = Yb / np.linalg.norm(Yb)
            Xb = np.cross(Yb, Zb)
            
            # Compute the rotation matrix associated with the body frame
            R[:, 0] = Xb 
            R[:, 1] = Yb
            R[:, 2] = Zb 

        return (X, Y, Z, W, R)

