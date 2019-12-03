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

PI = math.pi

# Lanes defined at 4, 0, -4

class PySimulator:

    def __init__(self, args, SimParams, NPC_traj):
        self.args = args
        # num_vehicles is only 2 for now
        self.NPC_dict = {}
        self.patches = []
        pdb.set_trace()
        self.num_vehicles = SimParams.num_vehicles
        self.navigation_agent = None
        self.NPC_states = NPC_states

        # Plot parameters
        self.fig = figure(figsize=(25,5))
        self.ax = fig.add_subplot(111)
        self.ax.axis('equal')
        self.ax.set_xlim(0, SimParams.map_lengthx)
        self.ax.set_ylim(-SimParams.map_lengthy, SimParams.map_lengthy)
        self.ax.axhline(y=SimParams.lane1, c='k', lw='4')
        self.ax.axhline(y=SimParams.lane2, c='k', lw='2', ls='--')
        self.ax.axhline(y=SimParams.lane3, c='k', lw='4')

        # Ego vehicle is first
        for i in range(num_vehicles):
            self.NPC_dict[i] = PolyRect(SimParams.car_dims)
            self.patches.append(self.NPC_dict[i].getPatch(self.ax))
        
        # May need to add loop here
        self.ax.add_patch(self.patches[0])
        self.ax.add_patch(self.patches[1])

    def create_global_plan(self):
        y = 2
        self.plan_ilqr = []
        for i in range(0, SimParams.map_length):
            self.plan_ilqr.append(np.array([i, y]))
        self.plan_ilqr = np.array(self.plan_ilqr)
        self.ax.axhline(y=y, c='r', lw='4')

    def init_sim(self):
        return self.patches[0], self.patches[1],

    def get_ego_states(self):
        ego_states = np.array([[self.current_ego_state[0], self.current_ego_state[1],                         0],
                               [self.current_ego_state[2],                         0,                         0],
                               [                        0,                         0, self.current_ego_state[3]],
                               [                        0,                         0,                         0],
                               [                        0,                         0,                         0]])
        return ego_states

    def get_npc_states(self):
        # vehicle_states = []
        # for n in range(len(self.vehicles_list)):
        #     vehicle_states.append(np.array([vehicle_transform.location.x,
        #                                     vehicle_transform.location.y,
        #                                     vehicle_velocity.x,,
        #                                     vehicle_transform.rotation.yaw]))
        np.zeros((4, 100))

    def create_ilqr_agent(self):
        self.navigation_agent = iLQR(self.args, self.get_npc_bounding_box())
        self.navigation_agent.set_global_plan(self.plan_ilqr)

    def run_step_ilqr(self):
        assert self.navigation_agent != None, "Navigation Agent not initialized"

        while True:
            counter = 0
            desired_path, local_plan, control = self.navigation_agent.run_step(self.get_ego_states(), self.get_npc_states())

        return control[:, 0]
 
    def animate(self,i):
        # Get new ego patch
        control = self.run_step_ilqr()
        self.current_ego_state = self.run_model_simulation(self.current_ego_state, control)
        self.NPC_dict[0].createCuboid([self.current_ego_state[0], self.current_ego_state[1], self.current_ego_state[3]]) # Update ego vehicle patch
        self.patches[0].set_xy(self.NPC_dict[0].getCorners()) # Update ego vehicle patch

        # Get new NPC patch
        self.NPC_dict[1].createCuboid(self.NPC_states[i])
        self.patches[1].set_xy(self.NPC_dict[1].getCorners())

        return self.patches[0], self.patches[1],

    def run_simulation(self):
        anim = animation.FuncAnimation(self.fig, self.animate,
                               init_func=self.init_sim,
                               frames=self.SimParams.sim_time,
                               interval=1000,
                               blit=True)
        plt.show()

    def run_model_simulation(self, state, control):
        """
        Find the next state of the vehicle given the current state and control input
        """
        # Clips the controller values between min and max accel and steer values
        control[0] = np.clip(control[0], SimParams.accel_min, SimParams.accel_max)
        control[1] = np.clip(control[1], state[2]*tan(SimParams.steer_min)/SimParams.wheelbase, state[2]*tan(SimParams.steer_max)/SimParams.wheelbase)
        
        Ts = SimParams.dt
        next_state = np.array([state[0] + cos(state[3])*(state[2]*Ts + (control[0]*Ts**2)/2),
                               state[1] + sin(state[3])*(state[2]*Ts + (control[0]*Ts**2)/2),
                               np.clip(state[2] + control[0]*Ts, 0.0, SimParams.max_speed),
                              (state[3] + control[1]*Ts)%(2*np.pi)])  # wrap angles between 0 and 2*pi
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
    max_speed = 180/3.6
    wheelbase = 2.94
    steer_min = -1.0
    steer_max = 1.0
    accel_min = -5.5
    accel_max = 3.0




if __name__ == "__main__":
    argparser = argparse.ArgumentParser(description='CARLA CILQR')
    add_arguments(argparser)
    args = argparser.parse_args()

    NPC_init = np.array([0, 2, 0])
    NPC_state = []
    # for i in range(0, 10, 0.1):
    #     NPC_state.append(np.array([NPC_init[0]+i, NPC_init[0], i/SimParams.dt, NPC_init[0]]))
    
    num_vehicles = 2
    pysim = PySimulator(args, SimParams, NPC_state)
