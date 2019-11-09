import numpy as np 
import scipy.integrate as integrate
from vehicle_model import Model

class iLQR():
    def __init__(self, timestep, horizon):
        self.Ts = timestep
        self.N = horizon
        raise NotImplementedError

    def forward_pass(self):
        raise NotImplementedError

    def backward_pass(self):
        raise NotImplementedError

    def run_step(self):
        raise NotImplementedError