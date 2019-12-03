import argparse
import logging
import os
import platform
import pdb
import math
import sys
import time

import numpy as np
from matplotlib import animation
import matplotlib.pyplot as plt
import matplotlib.patches
from sklearn.preprocessing import normalize

try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
except IndexError:
    print("Cannot add the common path {}".format(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from arguments import add_arguments
from PolyRect import PolyRect
from ilqr.iLQR import iLQR

# Add lambda functions
cos = lambda a : np.cos(a)
sin = lambda a : np.sin(a)
tan = lambda a : np.tan(a)

PI = math.pi

# Lanes defined at 4, 0, -4

class PySimulator:

    def __init__(self, args, SimParams, NPC_states):
        self.args = args
        # num_vehicles is only 2 for now
        self.NPC_dict = {}
        self.patches = []

        self.simparams = SimParams
        self.num_vehicles = self.simparams.num_vehicles
        self.navigation_agent = None
        self.NPC_states = NPC_states
        self.current_ego_state = self.simparams.start_state

        # Plot parameters
        self.fig = plt.figure(figsize=(25,5))
        self.ax = self.fig.add_subplot(111)
        self.ax.axis('equal')
        self.ax.set_xlim(0, self.simparams.map_lengthx)
        self.ax.set_ylim(-self.simparams.map_lengthy, self.simparams.map_lengthy)
        self.ax.axhline(y=self.simparams.lane1, c='k', lw='4')
        self.ax.axhline(y=self.simparams.lane2, c='k', lw='2', ls='--')
        self.ax.axhline(y=self.simparams.lane3, c='k', lw='4')

        # Plot Local Plan
        self.x_local_plan = []
        self.y_local_plan = []
        self.local_plan_plot, = plt.plot([], [], 'go')

        self.create_ilqr_agent()

        # Ego vehicle is first
        for i in range(num_vehicles):
            self.NPC_dict[i] = PolyRect(self.simparams.car_dims)
            self.patches.append(self.NPC_dict[i].getPatch(self.ax))
        
        # May need to add loop here
        self.ax.add_patch(self.patches[0])
        self.ax.add_patch(self.patches[1])

    def create_global_plan(self):
        y = self.simparams.desired_y
        self.plan_ilqr = []
        for i in range(0, self.simparams.map_lengthx):
            self.plan_ilqr.append(np.array([i, y]))
        self.plan_ilqr = np.array(self.plan_ilqr)
        self.ax.axhline(y=y, c='r', lw='2')

    def init_sim(self):
        return self.patches[0], self.patches[1], self.local_plan_plot,

    def get_ego_states(self):
        ego_states = np.array([[self.current_ego_state[0], self.current_ego_state[1],                         0],
                               [self.current_ego_state[2],                         0,                         0],
                               [                        0,                         0, self.current_ego_state[3]],
                               [                        0,                         0,                         0],
                               [                        0,                         0,                         0]])
        return ego_states

    def get_npc_bounding_box(self):
        return self.simparams.car_dims

    def get_npc_states(self):
        # vehicle_states = []
        # for n in range(len(self.vehicles_list)):
        #     vehicle_states.append(np.array([vehicle_transform.location.x,
        #                                     vehicle_transform.location.y,
        #                                     vehicle_velocity.x,,
        #                                     vehicle_transform.rotation.yaw]))
        np.zeros((4, 100))

    def create_ilqr_agent(self):
        self.create_global_plan()
        self.navigation_agent = iLQR(self.args, self.get_npc_bounding_box())
        self.navigation_agent.set_global_plan(self.plan_ilqr)

    def run_step_ilqr(self):
        assert self.navigation_agent != None, "Navigation Agent not initialized"

        desired_path, local_plan, control = self.navigation_agent.run_step(self.get_ego_states(), self.get_npc_states())
        print("Controller: Acc {} Steer: {}".format(control[0, 0], control[1, 0]))

        return desired_path, local_plan, control[:, 0]
 
    def animate(self, i):
        # Get new ego patch
        desired_path, local_plan, control = self.run_step_ilqr()
        self.current_ego_state = self.run_model_simulation(self.current_ego_state, control)
        self.NPC_dict[0].createCuboid([self.current_ego_state[0], self.current_ego_state[1], self.current_ego_state[3]]) # Update ego vehicle patch
        self.patches[0].set_xy(self.NPC_dict[0].getCorners()) # Update ego vehicle patch

        # Get new NPC patch
        # pdb.set_trace()
        self.NPC_dict[1].createCuboid([self.NPC_states[i, 0], self.NPC_states[i, 1], self.NPC_states[i, 3]])
        self.patches[1].set_xy(self.NPC_dict[1].getCorners())

        # Get local plan
        self.x_local_plan = local_plan[0, :]
        self.y_local_plan = local_plan[1, :]
        self.local_plan_plot.set_data(self.x_local_plan, self.y_local_plan)

        return self.patches[0], self.patches[1], self.local_plan_plot,

    def run_simulation(self):
        anim = animation.FuncAnimation(self.fig, self.animate,
                               init_func=self.init_sim,
                               frames=self.simparams.sim_time,
                               interval=1000,
                               blit=True)
        plt.show()

    def run_model_simulation(self, state, control):
        """
        Find the next state of the vehicle given the current state and control input
        """
        # Clips the controller values between min and max accel and steer values
        control[0] = np.clip(control[0], self.simparams.accel_min, self.simparams.accel_max)
        control[1] = np.clip(control[1], state[2]*tan(self.simparams.steer_min)/self.simparams.wheelbase, state[2]*tan(self.simparams.steer_max)/self.simparams.wheelbase)
        
        Ts = self.simparams.dt
        next_state = np.array([state[0] + cos(state[3])*(state[2]*Ts + (control[0]*Ts**2)/2),
                               state[1] + sin(state[3])*(state[2]*Ts + (control[0]*Ts**2)/2),
                               np.clip(state[2] + control[0]*Ts, 0.0, self.simparams.max_speed),
                              (state[3] + control[1]*Ts)%(2*np.pi)])  # wrap angles between 0 and 2*pi
        print("Next state {}".format(next_state))
        return next_state

class SimParams:
    dt = 0.1
    sim_time = 100
    map_lengthx = 50
    map_lengthy = 50
    lane1 = 4
    lane2 = 0
    lane3 = -4
    num_vehicles = 1

    ## Car Parameters
    car_dims = np.array([2, 1])
    start_state = np.array([10, 1, 0, 0])
    max_speed = 180/3.6
    wheelbase = 2.94
    steer_min = -1.0
    steer_max = 1.0
    accel_min = -5.5
    accel_max = 3.0
    desired_y = 2




if __name__ == "__main__":
    argparser = argparse.ArgumentParser(description='CARLA CILQR')
    add_arguments(argparser)
    args = argparser.parse_args()

    NPC_start = np.array([5, -2, 0])
    NPC_control = np.ones((2, SimParams.sim_time))
    NPC_traj = []
    for i in np.linspace(0, 10, SimParams.sim_time):
        NPC_traj.append(np.array([NPC_start[0]+i, NPC_start[1], 0.1/SimParams.dt, NPC_start[2]]))
    NPC_traj = np.array(NPC_traj)
    
    num_vehicles = 2
    pysim = PySimulator(args, SimParams, NPC_traj)
    pysim.run_simulation()
