#!/usr/bin/env python3

import numpy as np
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.planning import DirectTranscription
from pydrake.solvers import MathematicalProgram, Solve

#########
# Setup #
#########

time_step = 0.01
N = 400  # Number of transcribed samples (t_f = N * time_step)
w_goal = 100.0  # Final state weighting
x_init = [0.0, 0.0, 0.0, 0.0]  # Initial state [x, θ, xd, θd]
x_final = [0.0, np.pi, 0.0, 0.0]  # Final state [x, θ, xd, θd]

############
# Solution #
############

cart_pole = MultibodyPlant(time_step=time_step)
parser = Parser(cart_pole).AddModelsFromUrl(
    url="package://drake/examples/multibody/cart_pole/cart_pole.sdf"
)
cart_pole.Finalize()
context = cart_pole.CreateDefaultContext()

trajopt = DirectTranscription(
    cart_pole,
    context,
    N,
    input_port_index=cart_pole.get_actuation_input_port().get_index(),
)

trajopt.prog().AddBoundingBoxConstraint(x_init, x_init, trajopt.initial_state())
trajopt.prog().AddQuadraticErrorCost(w_goal * np.eye(4), x_final, trajopt.final_state())

sol = Solve(trajopt.prog())
t_star = trajopt.GetSampleTimes(sol)
x_star = trajopt.GetStateSamples(sol)
u_star = trajopt.GetInputSamples(sol)

############
# Plotting #
############

import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Spool out the optimized state
X_WC = np.zeros((N, 3))
X_WP = np.zeros((N, 3))
for i in range(0, N):
    context.SetDiscreteState(x_star[:, i])
    X_WC[i, :] = cart_pole.EvalBodyPoseInWorld(
        context, cart_pole.GetBodyByName("Cart")
    ).translation()
    X_WP[int, :] = cart_pole.EvalBodyPoseInWorld(
        context, cart_pole.GetBodyByName("Pole")
    ).translation()

class MpcAnimation:
    def __init__(self, ax):
        self.t0 = time.time()
        (self.line,) = ax.plot([], [], color="k", alpha=0.5)
        self.particle = ax.scatter([], [])
        self.target = ax.scatter([], [])

    def update(self, frame):
        t = time.time() - self.t0
        i = int(t / cart_pole.time_step()) % N
        self.line.set_data([X_WC[i, 0], X_WP[i, 0]], [X_WC[i, 2], X_WP[i, 2]])
        self.particle.set_offsets([X_WC[i, 0], X_WC[i, 2]])
        self.target.set_offsets([X_WP[i, 0], X_WP[i, 2]])
        return self.line, self.particle, self.target

fig = plt.figure()
ax = fig.add_subplot()
ax.set_aspect("equal", "box")

ax.set_xlim(-3.0, 3.0)
ax.set_ylim(-1.0, 1.0)

mpc = MpcAnimation(ax)
ax.plot(X_WP[:, 0], X_WP[:, 2], color="k", linestyle="--", alpha=0.1)
ani = FuncAnimation(fig, mpc.update, None, interval=10, blit=True)
plt.show()
