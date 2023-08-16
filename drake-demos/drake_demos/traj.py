import numpy as np
from pydrake.math import eq
from pydrake.solvers import MathematicalProgram, Solve
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import SpatialInertia, PlanarJoint

import matplotlib.pyplot as plt

def f():
    # Plant is stateless
    plant = MultibodyPlant(time_step=0)
    brick = plant.AddRigidBody("brick", SpatialInertia.SolidSphereWithMass(mass=1, radius=0.25))
    plant.AddJoint(PlanarJoint("joint", plant.world_frame(), brick.body_frame()))
    plant.Finalize()
    # Context is stateful
    ctx = plant.CreateDefaultContext()

    # Number of steps in the future to optimize over
    N = 500

    # min 0.5 * xQx' + bx is the program we're trying to describe in code
    # s.t. Ax = b
    #      Cx <= d

    prog = MathematicalProgram()
    q = prog.NewContinuousVariables(N, plant.num_positions(), "q")
    qd = prog.NewContinuousVariables(N, plant.num_positions(), "qd")
    qdd = prog.NewContinuousVariables(N, plant.num_positions(), "qdd")

    # Acceleration and velocity constraints
    prog.AddBoundingBoxConstraint(-0.5, 0.5, qdd)
    prog.AddBoundingBoxConstraint(-1, 1, qd)

    # Initial conditions
    # Our initial q has to be zero
    prog.AddConstraint(eq(q[0], [0, 0, 0]))
    # Our initial qd has to be zero (we're not moving)
    prog.AddConstraint(eq(qd[0], [-1.0, 0, 0]))

    # Dynamics constraints, ie describe how the variables are related
    # Describe what acceleration and velocity are but you could also have
    # kinematic constraints
    dt = 0.01
    for i in range(1, N):
        # q[i] = q[i - 1] + dt * qd[i - 1]
        prog.AddConstraint(eq(q[i], q[i - 1] + dt * qd[i - 1]))
        # qd[i] = qd[i - 1] + dt * qdd[i - 1]
        prog.AddConstraint(eq(qd[i], qd[i - 1] + dt * qdd[i - 1]))


    # Try to get to the goal
    # Q * (x - x_goal)
    # Weighting of position objective
    w_pos = 100
    prog.AddQuadraticErrorCost(w_pos * np.eye(plant.num_positions()), [2, 2, 0], q[-1, :])
    # Weigh velocity objective much much less
    w_vel = 1.0
    prog.AddQuadraticErrorCost(w_vel * np.eye(plant.num_positions()), [0, 0, 0], q[-1, :])
    result = Solve(prog)

    # Optimal trajectory
    q_opt = result.GetSolution(q)
    plt.plot(q_opt[:, 0], q_opt[:, 1])
    plt.show()

if __name__ == "__main__":
    print(f())
