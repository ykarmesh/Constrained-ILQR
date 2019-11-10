import numpy as np 
import scipy.integrate as integrate

from vehicle_model import Model
from local_planner import LocalPlanner
from constraints import Constraints


class iLQR():
    def __init__(self, args):
        self.args = args
        self.Ts = args.timestep
        self.N = args.horizon
        self.tol = args.tol
        
        self.local_planner = LocalPlanner(args)
        self.vehicle_model = Model(args)
        self.constraints = Constraints(args)
        raise NotImplementedError

    def forward_pass(self):
        raise NotImplementedError

    def backward_pass(self):
        raise NotImplementedError

    def run_step(self):
        raise NotImplementedError