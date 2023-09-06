#!/bin/env python3 
import mujoco
#import mujoco.viewer
import mujoco_viewer
import os
import time
import numpy as np

from pydrake.math import eq
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import SpatialInertia, PlanarJoint, RevoluteJoint, PrismaticJoint
from pydrake.planning import DirectTranscription
from pydrake.solvers import MathematicalProgram, Solve
from pydrake.symbolic import sin, cos, pow

class mpc:
    def __init__(self, model, data):
        self.dt = 0.01
        self.N = 25
        self.max_acc = 200.0
        # [x, theta, xd, thetad]
        self.goal = np.zeros((4))
        self.goal[1] = np.pi # Want the pole to be straight up, still
        self.plant = MultibodyPlant(time_step = self.dt)
        # Todo: get the plant dynamics from the model
        self.cart_mass = 1
        self.pole_mass = 0.1
        self.pole_length = 0.3
        self.g = 9.81
        cart = self.plant.AddRigidBody(
            "Cart",
            SpatialInertia.SolidBoxWithMass(
                mass = self.cart_mass, 
                lx = 1,
                ly = 0.5,
                lz = 1
            )
        )
        pole = self.plant.AddRigidBody(
            "Pole",
            SpatialInertia.SolidCylinderWithMass(
                mass = self.pole_mass, 
                radius = 0.1,
                length = self.pole_length,
                unit_vector = [0, -1, 0]
            )
        )
        # Let cart slide around in xy axis
        self.plant.AddJoint(
            PrismaticJoint(
                "cart_slide", 
                self.plant.world_frame(), 
                cart.body_frame(),
                [1, 0, 0]
            )
        )
        self.plant.AddJoint(
            RevoluteJoint(
                "pole_to_cart",
                cart.body_frame(),
                pole.body_frame(),
                [0, 0, 1]
            )
        )
        self.plant.Finalize()
        self.context = self.plant.CreateDefaultContext()
   
    def run_mpc(self, model, data):
        self.prog = MathematicalProgram()
        self.q = self.prog.NewContinuousVariables(self.N, self.plant.num_positions(), "q") # x (m)
        self.qd = self.prog.NewContinuousVariables(self.N, self.plant.num_velocities(), "qd") # v (m/s)
        self.qdd = self.prog.NewContinuousVariables(self.N, 1, "qdd") # a (m/s/s) (force is only applied to base of cart)
        # Force Constraints
        self.prog.AddBoundingBoxConstraint(-self.max_acc, self.max_acc, self.qdd)

        # Initial Condition Constraints
        self.prog.AddConstraint(eq(self.q[0], data.qpos))
        self.prog.AddConstraint(eq(self.qd[0], data.qvel))
        # Dynamic Constraints
        # x is q[:, 0]
        # theta is q[:, 1]
        for i in range(1, self.N):
            # x dot (velocity)
            self.prog.AddConstraint(
                eq(self.q[i], 
                   self.q[i-1] + self.qd[i-1] * self.dt
                )
            )
            # x dot dot
            self.prog.AddConstraint(
                eq(self.qd[i][0],
                   (self.qd[i-1][0] + (self.qdd[i-1] + self.pole_mass * sin(self.q[i-1][1]) * (self.pole_length * pow(self.qd[i-1][1], 2) + self.g * cos(self.q[i-1][1]))) / (self.cart_mass + self.pole_mass * pow(sin(self.q[i-1][1]), 2)) * self.dt)
                )
            )
            # theta dot dot
            self.prog.AddConstraint(
                eq(self.qd[i][1], 
                   (self.qd[i-1][1] + (-self.qdd[i-1] * cos(self.q[i-1][1]) - self.pole_mass * self.pole_length * pow(self.qd[i-1][1], 2) * cos(self.q[i-1][1]) * sin(self.q[i-1][1]) - (self.cart_mass + self.pole_mass) * self.g * sin(self.q[i-1][1])) / (self.pole_length * (self.cart_mass + self.pole_mass * pow(sin(self.q[i-1][1]), 2))) * self.dt)
                )
            )

        # Goal cost
        for i in range(self.N):
            w_angle = 10.0
            w_cartpos = 10.0
            w_velocity = 0.01
            self.prog.AddQuadraticErrorCost(np.diag([w_cartpos, w_angle]), self.goal[0:2], self.q[i, :])
            self.prog.AddQuadraticErrorCost(w_velocity * np.eye(2), self.goal[2:], self.qd[i, :])
        result = Solve(self.prog)
        self.qdd_opt = result.GetSolution(self.qdd)
        print("Soln:", self.qdd_opt[0])

        data.qfrc_applied = [self.qdd_opt[0], 0]


model_path = os.path.expanduser("./cart_pole.xml")
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

mujoco.mj_resetData(model, data)
data.qpos[1] = np.pi - 0.1
print("data ctrl:", data.ctrl)
myMPC = mpc(model, data)
mujoco.set_mjcb_control(myMPC.run_mpc)

#viewer = mujoco.viewer.launch_passive(model, data)
viewer = mujoco_viewer.MujocoViewer(model, data)
time.sleep(0.5)
for _ in range(10000):
    #if viewer.is_running():
    if viewer.is_alive:
        mujoco.mj_step(model, data) # x_(t+h) = f(x_t)
        #viewer.sync()
        viewer.render()
    else:
        break

viewer.close()
