import numpy as np 
import scipy.integrate as integrate

from ilqr.vehicle_model import Model
from ilqr.local_planner import LocalPlanner
from ilqr.constraints import Constraints


class iLQR():
    def __init__(self, args, obstacle_bb, verbose=False):
        self.args = args
        self.Ts = args.timestep
        self.N = args.horizon
        self.tol = args.tol
        self.obstacle_bb = obstacle_bb
        self.verbose = verbose
        
        self.global_plan = None
        self.local_planner = LocalPlanner(args)
        self.vehicle_model = Model(args)
        # self.constraints = Constraints(args)
    
    def set_global_plan(self, global_plan):
        self.global_plan = global_plan
        self.local_planner.set_global_planner(self.global_plan)

    def forward_pass(self):
        raise NotImplementedError

    def backward_pass(self):
        raise NotImplementedError

    def run_step(self, ego_state, npc_states):
        assert self.global_plan is not None, "Set a global plan in iLQR before starting run_step"

        self.local_planner.set_ego_state(ego_state)
        path = self.local_planner.get_local_plan_waypoints()
        return path, 0.0