#!/usr/bin/env python3

import numpy as np
import pydrake.math
from pydrake.solvers import MathematicalProgram, Solve

# MPC or Trajectory Optimization?
use_mpc = True

# Configuration
dt = 0.1
num_time_steps = 20 if use_mpc else 50
max_velocity = 0.25
max_omega = 1.5708

# Objective weighting
w_pos = 1000.0
w_theta = 1.0

def solve_mpc(x0, theta0, xf, thetaf):
    prog = MathematicalProgram()

    # Position, velocity
    x = prog.NewContinuousVariables(num_time_steps, 2, "x")
    xd = prog.NewContinuousVariables(num_time_steps, 1, "xd")
    theta = prog.NewContinuousVariables(num_time_steps, 1, "theta")
    omega = prog.NewContinuousVariables(num_time_steps, 1, "omega")

    # Velocity constraint
    prog.AddBoundingBoxConstraint(0, max_velocity, xd)
    prog.AddBoundingBoxConstraint(-max_omega, max_omega, omega)

    # Dynamic constraints
    for i in range(1, num_time_steps):
        prog.AddQuadraticCost(np.eye(1), [0.0], xd[i])
        prog.AddQuadraticCost(np.eye(1), [0.0], omega[i])
        prog.AddConstraint(x[i][0] == x[i - 1][0] + xd[i - 1][0] * pydrake.math.cos(theta[i - 1][0]) * dt)
        prog.AddConstraint(x[i][1] == x[i - 1][1] + xd[i - 1][0] * pydrake.math.sin(theta[i - 1][0]) * dt)
        prog.AddConstraint(pydrake.math.eq(theta[i], theta[i - 1] + omega[i - 1] * dt))

    # Initial conditions
    prog.AddConstraint(pydrake.math.eq(x[0], x0))
    prog.AddConstraint(pydrake.math.eq(theta[0], theta0))

    # Goal cost
    prog.AddQuadraticErrorCost(w_pos * np.eye(2), xf, x[-1])
    prog.AddQuadraticErrorCost(w_theta * np.eye(1), thetaf, theta[-1])

    # Solve!
    result = Solve(prog)

    return result.GetSolution(x), result.GetSolution(xd), result.GetSolution(theta), result.GetSolution(omega)


import matplotlib
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.animation import FuncAnimation

fig = plt.figure()
ax = fig.add_subplot()
ax.set_aspect("equal", "box")

ax.set_xlim(-1.5, 1.5)
ax.set_ylim(-1.5, 1.5)

particle = ax.scatter([], [])
target = ax.scatter([], [])
(points,) = ax.plot([], [], color="k", alpha=0.2, marker=".")
(heading, ) = ax.plot([], [], color="k")

class MpcAnimation:
    def __init__(self, x0, theta0):
        self.t = 0.0
        self.x = x0
        self.theta = theta0
        self.xf = [0.0, 1.0]
        self.thetaf = [3.14159/2]

        self.x_calc = None
        self.xd_calc = None
        self.t_calc = None
        self.o_calc = None

    def update(self, *args):
        dt = 0.01

        x, xd, theta, omega = solve_mpc(self.x, self.theta, self.xf, self.thetaf)

        self.x_calc = x
        self.xd_calc = xd

        self.t_calc = theta
        self.o_calc = omega

        # If doing MPC, apply the first control command and integrate the
        # dynamics forward. Otherwise doing TrajOpt, just display the full
        # trajectory from the initial configuration.
        if use_mpc:
            self.x[0] += xd[0]*np.cos(self.theta) * dt
            self.x[1] += xd[0]*np.sin(self.theta) * dt
            self.theta += omega[0] * dt

        particle.set_offsets(x[0, :])
        target.set_offsets([self.xf[0], self.xf[1]])
        points.set_data(x[:, 0], x[:, 1])

        heading.set_data(np.array([[x[0, 0]], [x[0, 0] + 0.1*np.cos(self.theta[0])]]), np.array([[x[0, 1]], [x[0, 1] + 0.1*np.sin(self.theta[0])]]))
        return particle, target, points, heading


x0 = np.array([1.0, 0.0])
theta0 = np.array([3.14159])
mpc = MpcAnimation(x0, theta0)

def on_cursor(ev):
    if ev.xdata and ev.ydata:
        mpc.xf = [ev.xdata, ev.ydata]


fig.canvas.mpl_connect("motion_notify_event", on_cursor)

ani = FuncAnimation(fig, mpc.update, None, interval=0.01, blit=True)

# manager = plt.get_current_fig_manager()
# manager.full_screen_toggle()

plt.show()
