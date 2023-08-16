import numpy as np
from pydrake.math import eq
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import SpatialInertia, PrismaticJoint, RevoluteJoint
from pydrake.solvers import MathematicalProgram, Solve
import matplotlib.pyplot as plt
import time
from pydrake.symbolic import sin, cos, pow

def main():
    dt = 0.01
    N = 100
    pole_length = 1
    cart_mass = 1
    pole_mass = 0.1
    g = 9.80665

    plant = MultibodyPlant(time_step = 0)
    cart = plant.AddRigidBody("Cart", SpatialInertia.SolidBoxWithMass(mass=cart_mass,lx=1,ly=0.5, lz=1))
    plant.AddJoint(PrismaticJoint("cart_track", plant.world_frame(), cart.body_frame(), [1, 0, 0]))
    pole = plant.AddRigidBody("Pole", SpatialInertia.SolidCylinderWithMass(pole_mass, 0.3, pole_length, [0, -1, 0]))
    plant.AddJoint(RevoluteJoint("cart-pole",cart.body_frame(), pole.body_frame(), [0, 0, 1]))
    plant.Finalize()

    prog = MathematicalProgram()

    q = prog.NewContinuousVariables(N, plant.num_positions(), "q")
    qd = prog.NewContinuousVariables(N, plant.num_velocities(), "qd")
    F = prog.NewContinuousVariables(N, 1, "F")

    # Force constraints
    prog.AddBoundingBoxConstraint(-100, 100, F)

    ICs = np.zeros((2,2))
    ICs[0][1] = np.pi

    prog.AddConstraint(eq(q[0], ICs[0, :]))
    prog.AddConstraint(eq(qd[0], ICs[1, :]))

    # Don't go off screen
    # prog.AddBoundingBoxConstraint(-100, 100, q[:, 0])

    # Dynamic constraints
    for i in range(1, N):
        prog.AddConstraint(eq(q[i], q[i-1] + qd[i-1] * dt))

        # x accel
        prog.AddConstraint(eq(qd[i][0], (qd[i-1][0] + (F[i-1] + pole_mass * sin(q[i-1][1]) * (pole_length * pow(qd[i-1][1],2) + g * cos(q[i-1][1]))) / (cart_mass + pole_mass * pow(cos(q[i-1][1]), 2)) * dt)))

        # angular accel
        prog.AddConstraint(eq(qd[i][1], (qd[i-1][1] + (-F[i-1] * cos(q[i-1][1]) - pole_mass * pole_length * pow(qd[i-1][1], 2) * cos(q[i-1][1]) * cos(q[i-1][1]) - (cart_mass + pole_mass) * g * cos(q[i-1][1])) / (pole_length * (cart_mass + pole_mass * pow(cos(q[i-1][1]),2))) * dt)))
    
    goal = np.zeros((2, 2))
    for i in range(0, N):
        w_angle = 100.0
        w_cartpos = 10.
        w_velocity = 0.
        prog.AddQuadraticErrorCost(np.diag([w_cartpos, w_angle]), goal[0, :], q[i,:])
        prog.AddQuadraticErrorCost(w_velocity * np.eye(2), goal[0, :], qd[i,:])
    
    result = Solve(prog)
    q_opt = result.GetSolution(q)
    qd_opt = result.GetSolution(qd)
    F_opt = result.GetSolution(F)
    
    return q_opt, qd_opt, F_opt


if __name__ == '__main__':
    q_opt, qd_opt, F_opt = main()
    figure = plt.figure()
    plt.plot(q_opt[:,0])
    plt.title("Cart Position")
    figure = plt.figure()
    plt.plot(q_opt[:,1])
    plt.title("Angle")
    force_figure = plt.figure()
    plt.plot(F_opt)
    plt.title("Force")
    xy_figure = plt.figure()
    plt.plot(np.sin(q_opt[:,1]) + q_opt[:, 0], np.cos(q_opt[:,1]))
    plt.title("Pole y vs x position")
    plt.show()