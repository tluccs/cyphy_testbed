#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jun  7 18:11:29 2019

@author: rt-2pm2
"""
import numpy as np
import numpy.polynomial.polynomial as pl

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '.')))

import trjgen.trjgen_core as trjg

class PwPoly :

    ## Constructor
    def __init__(self, waypoints=None, knots=None, degree=None):
        
        if knots is not None:
            # Knots (It's assumed that the knot of the first point is 0.0)
            self.knots = np.array(knots)

        if degree is not None:
            # Degree of the polynomial
            self.degree = degree

        if (waypoints is not None and knots is not None and degree is not None): 
            # Store the waypoints
            self.wp = waypoints
            # Interpolation problem
            (sol, null, _, coeff_m) = trjg.interpolPolys(waypoints, degree, knots, True)

            # Solution vector of the polynomial coefficient (1 row)
            self.coeff_v = np.array(sol)

            # Base of the null space of the interpolation solution
            self.null_b = np.array(null)

            # Matrix of polynomials coefficients
            self.coeff_m = np.array(coeff_m)

            # Number of pieces
            self.npieces = coeff_m.shape[0]


    # Evaluate the Piecewise polynomial
    def eval(self, t, der):
        """
        Evaluate the piecewise polynomial

        Args:
            t vector of time instants
            der derivative to evaluate
        """

        # Check whether the evaluation is requested on a scalar or on an
        # iterable
        try:
            # Iterable: t is a vector
            t = np.array(t)
            _ = iter(t)
            nsamples = t.size
            if (t[0] < 0.0) or (t[nsamples - 1] > self.knots[-1]):
                print("PwPoly.eval(): Trying to evaluate outside " +
                        "the time support of the piecewise polynomial")

            # Allocate the variable
            yout = np.zeros((nsamples), dtype = float)
            for (k, t_) in enumerate(t):
                i = self.find_piece(t_)
                # Evaluate the required polynomial derivative
                pder = pl.polyder(self.coeff_m[i, :], der)
                yout[k] = pl.polyval(t_ - self.knots[i], pder)
        except TypeError:
            # Non iterable: t is a single value
            if (t < 0.0) or (t > self.knots[-1]):
                print("Warning! PwPoly.eval(): Trying to evaluate outside " +
                        "the time support of the piecewise polynomial")

            i = self.find_piece(t)
            pder = pl.polyder(self.coeff_m[i, :], der)
            yout = pl.polyval(t - self.knots[i], pder)

        return yout

    def moveWps(self, new_waypoints):
        """
        Move the position of the waypoints
        """
        if (len(new_waypoints) != len(self.knots)):
            print("The lenght of the waypoints list should not change")
            return False;

        # Store the waypoints
        self.wp = new_waypoints

        # Interpolation problem
        (sol, null, _, coeff_m) = trjg.interpolPolys(self.wp, self.degree, self.knots, True)

        # Update the data
        # Solution vector of the polynomial coefficient (1 row)
        self.coeff_v = np.array(sol)

        # Base of the null space of the interpolation solution
        self.null_b = np.array(null)

        # Matrix of polynomials coefficients
        self.coeff_m = np.array(coeff_m)

        # Number of pieces
        self.npieces = coeff_m.shape[0]

        return True

    def moveKnots(self, new_knots):
        """
        Move the position of the knots
        """
        if (len(new_knots) != len(self.knots)):
            print("The lenght of the knots list should not change")
            return False;

        # Knots update
        self.knots = np.array(new_knots)

        # Interpolation problem
        (sol, null, _, coeff_m) = trjg.interpolPolys(self.wp, self.degree, self.knots, True)

        # Update the data
        # Solution vector of the polynomial coefficient (1 row)
        self.coeff_v = np.array(sol)

        # Base of the null space of the interpolation solution
        self.null_b = np.array(null)

        # Matrix of polynomials coefficients
        self.coeff_m = np.array(coeff_m)

        # Number of pieces
        self.npieces = coeff_m.shape[0]

        return True;

    ## Function to retrieve values

    def getWaypoints(self):
        """
        Returns the waypoints of the piecewise polynomial
        """
        wp = self.wp
        return np.array(wp)

    def getKnots(self):
        """
        Returns the waypoints of the piecewise polynomial
        """
        knots = self.knots
        return np.array(knots)

    def getCoeffMat(self):
        """
        Returns the coefficients of the piecewise polynomial
        as a matrix with each piece on a line
        """
        coeff = self.coeff_m
        return np.array(coeff)

    def getCoeffVect(self):
        """
        Returns the coefficients of the piecewise polynomial
        as a vector
        """
        coeff_v = self.coeff_v
        return coeff_v

    def loadFromData(self, M, Dt, npieces):
        """
        Load the polynomial from data
        """
        if (M.shape[0] == 1):
            self.coeff_v = M 
            self.coeff_m = M.reshape(npieces, -1)
        else:
            self.coeff_m = M
            self.coeff_v = M.reshape(1, -1)

        self.npieces = npieces
        
        self.degree = self.coeff_m.shape[1] - 1
        
        self.knots = np.zeros(Dt.size + 1)
        for i in range(Dt.size):
            self.knots[i + 1] = self.knots[i] + Dt[i]

    ## Private
    # Find the piece of the piecewies polynomial active at time t
    def find_piece(self, t):
        if (t < self.knots[0] or t > self.knots[-1]):
            return -1

        for i in range(self.npieces):
            if t >= self.knots[i] and t <= self.knots[i+1]:
                return i




