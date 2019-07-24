"""
Support for trajectory generation

deg: degree of the polynomial used for describing the trajectory
der: number of derivatives of the flat output used to define the trajecotry

The trajectory will be generated modeling the problem as an optimization
problem.

The constraints will be in the form:
    c' * A = b,
where "c" is the vector of polynomial coefficients, "A" is the matrix mapping
coefficients with flat oututs and "b" is the vector of constraints in the
flat output space.

The conditions for the polynomial will be provided with a matrix X

Where X[i][j] is the requirement for the i-derivative of the j-th waypoint.

[1 t t^2 t^3 ...] * c1 = p1(t)
[1 t t^2 t^3 ...] * c2 = p2(t)

@author: l.pannocchi@gmail.com
"""
import numpy as np
import scipy.linalg as linalg


## =================================================
## =================================================
def interpolPolys(X, deg, T, is_abstime):
    """
    Find the coefficients of the polynomials interpolating a given series
    of waypoints

    Args
        X:   Matrix of the constraints Nconstr x Nwaypoinst
        deg: Degree of the polynomial to fit to describe the pieces
        T: Knots points for the interpolation
        abstime: Flag to specify whether the t vector represents
                    durations or absolute time


    Output:
        sol:   Solution vector of the Ax = b problem (All coeff in a row)
        nullx: Null space of the A
        res:   Residuals of the interpolation problem
        polys: Matrix with the polynomial coefficients ([a_0 a_1 ...a_deg])
               for each piece of trajectory on the rows.
               The polynomial would be (a_0 + a_1*t + a_2*t^2 ...)

    """
    nCoeff = deg + 1
    [A, b] = buildInterpolationProblem(X, deg, T, is_abstime)

    nullx = np.zeros((3,3)) #linalg.null_space(A)

    [sol, res, _, _] = linalg.lstsq(A, b)

    npolys = int(len(sol) / nCoeff)

    polys = np.zeros((npolys, nCoeff), dtype=float)
    for i in range(npolys):
        polys[i, :] = sol[i * nCoeff: (i + 1) * nCoeff]

    return (sol, nullx, res, polys)


## =================================================
## =================================================
def buildInterpolationProblem(X, deg, T, is_abstime = True):
    """
    Build the interpolation problem A * x = b

    Args
        X:   Matrix of the constraints Nconstr x Nwaypoints
        deg; Degree of the polynomial to fit to describe the pieces
        T:  Vector with the time information (Either the waypoint pass time
                                              or the duration for each piece)
        abstime: Flag to specify whether the t vector represents
                    durations or absolute time


    Output:
        A: Matrix of the system
        b: Vector of the known terms
    """

    nCoef = deg + 1        # Number of coefficient to describe each polynomial
    nConstr = X.shape[0]   # Number of constraints (flat outputs)
    nWp = X.shape[1]       # Number of waypoints

    nEq = numOfEquations(X) # Number of equation of the interpolation problem

    # Length of the overall vector containing the coefficients
    # of the polynomials, that is, (nCoef * NumberOfPieces)
    nC = (deg + 1) * (nWp - 1)

    # Instantiate the output variables
    A = np.zeros((nEq, nC), dtype=float)
    b = np.zeros((nEq), dtype=float)

    # Define the time vector
    Dt = np.zeros((nWp - 1), dtype=float)

    if (is_abstime):
        for i in range(len(T) - 1):
            Dt[i] = T[i+1] - T[i]
    else:
        Dt = T

    if ((Dt < 0.0).any()):
        print("The time for completing a piece of trajectory \
              cannot be negative")
        return (A, b)


    counter = 0;
    for i in range(nWp):            # For each waypoint
        for j in range(nConstr):        # For each constraint
            # If not (First or Last)
            if (i > 0 and i < (nWp - 1)):
                # If specified wp
                if (not np.isnan(X[j,i])):
                    # Waypoint "i" is connects Polynomial i-1 and Polynomial i
                    b[counter] = X[j,i]

                    v = t_vec(Dt[i-1], deg)
                    A[counter, selIndex(i-1, nCoef)] = polyder(v, j)
                    counter = counter + 1

                    b[counter] = X[j,i]
                    v = t_vec(0, deg)
                    A[counter, selIndex(i, nCoef)] = polyder(v, j)
                # Only continuity constraint
                else:
                    b[counter] = 0

                    v = t_vec(Dt[i-1], deg)
                    A[counter, selIndex(i-1, nCoef)] = polyder(v, j)

                    v = t_vec(0, deg)
                    A[counter, selIndex(i, nCoef)] = -1 * polyder(v, j)

            # If First of Last
            else:
                if (i == 0):
                    b[counter] = X[j,i]
                    v = t_vec(0, deg)
                    A[counter, selIndex(i, nCoef)] = polyder(v, j)
                else:
                    b[counter] = X[j,i]
                    v = t_vec(Dt[i-1], deg)
                    A[counter, selIndex(i-1, nCoef)] = polyder(v, j)
            counter = counter + 1

    return (A,b)

## =================================================
## =================================================
def pp2file(Dt, polysX, polysY, polysZ, polysW, filename):
    """
    Save the polynomial coefficients to file
    The file will have a row for each piece and the row has the
    following format:

    Time_interval x_poly_coeff y_poly_coeff z_poly_coeff w_poly_coeff

    """
    f = open(filename, "w")

    # I consider all the polynomial to be the same size
    nPieces = polysX.shape[0]
    pollen = polysX.shape[1]

    f.write('\n')
    for i in range(nPieces):
        f.write('{:5f}, '.format(Dt[i]))
        for j in range(pollen):
            f.write('{:5f}, '.format(polysX[i,j]))
        for j in range(pollen):
            f.write('{:5f}, '.format(polysY[i,j]))
        for j in range(pollen):
            f.write('{:5f}, '.format(polysZ[i,j]))
        for j in range(pollen):
            f.write('{:5f}, '.format(polysW[i,j]))
        f.write('\n')

    f.close()



######################################################################
## Helper functions for polynomial generation


## =================================================
## =================================================
def t_vec(t, deg):
    """
    Generate the time polynomial vector [1 t t^2 t^3 ... t^deg]
    """
    v = np.ones((deg + 1), dtype=float)

    for i in range(1, deg+1):
        v[i] = v[i-1] * t

    return v;


## =================================================
## =================================================
def derivmat(deg):
    """
    Compute the matrix, which maps time polynomial vector in their
    derivative.
    """
    msize = deg + 1
    M = np.zeros((msize, msize), dtype=float)

    for i in range(deg):
        M[i + 1][i] = i + 1

    return M


## =================================================
## =================================================
def polyder(np_arr_t, n_der):
    """
    Compute the n_der derivative of a given time vector vector
    """
    # Compute the length of the array
    v_len = np.size(np_arr_t)

    if (n_der > v_len - 1):
        print("Derivative order higher than the degree of the polynomial!")
        return np.zeros((v_len), dtype = float)

    v = np_arr_t

    # Derivative matrix (vlen = deg + 1)
    M = derivmat(v_len - 1)

    for i in range(1,n_der + 1):
        v = np.matmul(M, v)

    return v


## =================================================
## =================================================
def constrMat(tBase, indConstrDer):
    """
    Build the constraint matrix A for the interpolation problem (Ax = s)
    Takes as input the time polynomial and the indexes of the derivative,
    ehich constitutes the object to be constrained.

    Ex.
    tBase = [1 t0 t0^2 t0^3]
    indConstrDer = [0 1 2]

    Defines the problem:
    [1  t0 t0^2  t0^3 ]
    [0  1  2*t0 3*t0^2] * c = s0    ==> A * c = s0
    [0  0   2    6*t0 ]

    """
    if (type(indConstrDer) == list):
        nconstr = len(indConstrDer)
    else:
        if (type(indConstrDer) == np.ndarray):
            nconstr = indConstrDer.size
        else:
            print("Wrong type for the index variable")
            return


    ncoeff = tBase.size

    A = np.zeros((nconstr, ncoeff), dtype = float)

    for i in range(nconstr):
        A[i, :] = polyder(tBase, indConstrDer[i]).transpose()

    return A


## =================================================
## =================================================
def numOfEquations(W):
    """
    Count the number of equation necessary to set up the interpolation problem
    for trajectory generation.

    The number of equations depends on the number and type of constraints.
    There are "specific constraints" where the polynomials are required to
    pass through given points of the flat output trajectory and
    "Smoothness constraints", which simply ask to have a smooth connection
    between pieces of trajectory described by sequential polynomials.

    The number of equation is (WP0 + WPend) + 2 x (Fixed Cnstr) + (Smooth Cnstr)

    Input
        W: Matrix of the constraints (Nconstraints x Nwaypoints)

    Ouput
        nCr: Number of equations required to set up the interpolation problem
    """

    nWr = W.shape[0]
    nWc = W.shape[1]

    # First and End condition (fixed points)
    nCr = 2 * nWr

    # Junction points of the trajecotry
    # For each fixed point condition we will get 2 conditions, whilst, for
    # the smooth constraints we will obtain only 1 condition.

    # Select the waypoints, which are shared between polynomial
    Wshared = W[:, 1: (nWc - 1)]
    fWshared = Wshared.flatten()

    # Count how many fixed point condition and smooth condition I have
    nshared_fix = np.count_nonzero(~np.isnan(fWshared))
    nshared_smooth = np.count_nonzero(np.isnan(fWshared))
    nCr += 2 * nshared_fix + nshared_smooth

    return nCr


## =================================================
## =================================================
def selIndex(i, numCoeff):
    """
    Generate the indexes to select a subvector made of pieces of
    length numCoeff
    """
    return range(numCoeff * i, numCoeff * (i + 1))
