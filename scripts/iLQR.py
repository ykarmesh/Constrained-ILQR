import numpy as np 
import scipy.integrate as integrate

class iLQR():
    def __init__(self):
        raise NotImplementedError

    def forward_pass(self):
        raise NotImplementedError

    def backward_pass(self):
        raise NotImplementedError

    def run_step(self):
        raise NotImplementedError