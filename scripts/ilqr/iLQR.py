import numpy as np 
import scipy.integrate as integrate

from vehicle_model import Model
from local_planner import LocalPlanner
from constraints import Constraints


class iLQR():
    def __init__(self, args, obstacle_bb):
        self.args = args
        self.Ts = args.timestep
        self.N = args.horizon
        self.tol = args.tol
        self.obstacle_bb = obstacle_bb
        
        self.local_planner = LocalPlanner(args)
        self.vehicle_model = Model(args)
        self.constraints = Constraints(args)
    
    def set_global_plan(self, global_plan):
        self.global_plan = global_plan
        self.local_planner.set_global_planner(self.global_plan)

    def forward_pass(self):
        raise NotImplementedError

    def backward_pass(self):
        raise NotImplementedError

    def run_step(self):
        raise NotImplementedError