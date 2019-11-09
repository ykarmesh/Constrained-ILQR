import numpy as np 

class Constraints:
    def __init__(self, args):
        self.args = args
    
    def get_Q(self):
        """
        Returns the state quadratic cost term (Q matrix) for the trajectory
        """
        return Q

    def get_R(self):
        """
        Returns the control quadratic cost term (R matrix) for the trajectory
        """
        return R

    def get_q(self):
        """
        Returns the state linear cost term (q vector) for the trajectory
        """
        return q

    def get_r(self):
        """
        Returns the control linear cost term (r vector) for the trajectory
        """
        return r

    def get_cost(self):
        """
        Returns the different cost terms for the trajectory
        This is the main function which calls all the other functions 
        """

        return Q, R, q, r
