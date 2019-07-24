#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Helper functions to plot with plotly/dash

@author: rt-2pm2
"""

import plotly as py
import plotly.graph_objs as go

import trjgen.trjgen_helpers as tjh


## =================================================
## =================================================
def plotTray_plotly(X, Y, Z, t):
    trace1 = go.Scatter3d(
            x = X[0, :],
            y = Y[0, :],
            z = Z[0, :],
            mode = 'markers',
            marker = dict(
                size = 4,
                color = t,
                colorscale = 'Viridis',
                opacity = 0.8,
                colorbar = dict (
                    thickness = 20,
                    len =  0.5,
                    x = 0.8,
                    y = 0.6
                    )
                ),
            name = '3D Path'
            )

    data = [trace1]
    layout = go.Layout(
            margin = dict(
                l = 0,
                r = 0,
                b = 0,
                t = 0
                ),
            scene={"aspectmode": "data",
                "xaxis": {"title": f"x [m]"},
                "yaxis": {"title": f"y [m]"},
                "zaxis": {"title": f"z [m]"}}
            )

    fig = go.Figure(data = data, layout = layout)
    py.offline.plot(fig, filename='traj.html')


## =================================================
## =================================================
def plotZb_plotly(X, Y, Z, V):
    data = [{
            "type": "cone",
            "x": X[0,:],
            "y": Y[0,:],
            "z": Z[0,:],
            "u": V[0,:],
            "v": V[1,:],
            "w": V[2,:],
            "sizemode": "absolute",
            "colorscale": 'Blues',
            "hoverinfo": "u+v+w",
            "sizeref": 10000
    }]
    
    #range = [0.0, 3.0]
    layout = go.Layout(
            margin = dict( l = 0, r = 0, b = 0, t = 0),
            scene={"aspectmode": "data",
                "xaxis": {"title": f"x [m]"},
                "yaxis": {"title": f"y [m]"},
                "zaxis": {"title": f"z [m]"}}

            )

    fig = go.Figure(data = data, layout = layout)
    py.offline.plot(fig,validate=False)


## =================================================
## =================================================
def plotThrustMargin(T, X, Y, Z, vehicle_mass, thrust_constr):
    """
    Plot the trajectory coordinates as a scatter plot evaluating also
    the thrust margin available for maneuvers.
    """

    (ffthrust, available_thrust) = tjh.getlimits(T, X, Y, Z, vehicle_mass, thrust_constr)
    perc_available = (available_thrust / thrust_constr) * 100.0
    if ((X.shape[1] < 3) or (Y.shape[1] < 3) or (Z.shape[1] < 3)):
        print("The trajectory should contain at least acceleration")
        return 0


    trace1 = go.Scatter3d(
        x = X[0, :],
        y = Y[0, :],
        z = Z[0, :],
        mode = 'markers',
        marker = dict(
            size = 4,
            color = perc_available,
            colorscale = 'Viridis',
            opacity = 0.8,
            colorbar = dict ( thickness = 20, len =  0.5, x = 0.8, y = 0.6)
            ),
        name = '3D Path with thrust free margin'
        )

    data = [trace1]
    layout = go.Layout(
            margin = dict( l = 0, r = 0, b = 0, t = 0),
            scene={"aspectmode": "data", "xaxis": {"title": f"x [m]", },
                "yaxis": {"title": f"x [m]", },
                "zaxis": {"title": f"z [m]", }}
            )
    fig = go.Figure(data = data, layout = layout)
    py.offline.plot(fig, filename="thrust_marging.html")

    return


