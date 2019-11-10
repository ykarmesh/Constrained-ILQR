import numpy as np 
import math

class Constraints:
    def __init__(self, args):
        self.args = args
        self.control_cost = np.array([[self.args.w_acc,                   0],
                                      [              0, self.args.w_yawrate]])

        self.state_cost = np.array([[self.args.w_pos, 0, 0, 0],
                                    [0, self.args.w_pos, 0, 0],
                                    [0, 0, self.args.w_vel, 0],
                                    [0, 0, 0,               0]])
    
    def get_state_cost(self):
        """
        Returns the state quadratic (Q matrix) and linear cost term (q matrix) for the trajectory
        """
        return Q, q

    def get_control_cost(self, state, control):
        """
        Returns the control quadratic (R matrix) and linear cost term (r vector) for the trajectory
        """
        P1 = np.array([[1],[0]])
        P2 = np.array([[0],[1]])

        for i in range(len(control)):
            # Acceleration Barrier Max
            c = (np.matmul(control[i].T, P1) - self.args.acc_limits[1])
            b, b_dot, b_ddot = self.barrier_function(self.args.q1_acc, self.args.q2_acc, c, P1)

            # Acceleration Barrier Min
            c = (self.args.acc_limits[0] - np.matmul(control[i].T, P1))
            b, b_dot, b_ddot = self.barrier_function(self.args.q1_acc, self.args.q2_acc, c, -P1)

            # Yawrate Barrier Max
            c = (np.matmul(control[i].T, P2) - state[i,2]*math.tan(self.args.steer_angle_limits[1])/self.args.wheelbase)
            b, b_dot, b_ddot = self.barrier_function(self.args.q1_yawrate, self.args.q2_yawrate, c, P2)

            # Yawrate Barrier Min
            c = (state[i,2]*math.tan(self.args.steer_angle_limits[0])/self.args.wheelbase - np.matmul(control[i].T, P2))
            b, b_dot, b_ddot = self.barrier_function(self.args.q1_yawrate, self.args.q2_yawrate, c, -P2)

        return R, r

    def get_cost(self):
        """
        Returns the different cost terms for the trajectory
        This is the main function which calls all the other functions 
        """

        return Q, R, q, r

    def barrier_function(self, q1, q2, c, c_dot):
        b = q1*np.exp(q2*c)
        b_dot = q1*q2*np.exp(q2*c)*c_dot
        b_ddot = q1*(q2**2)*np.exp(q2*c)*np.matmul(c_dot, c_dot.T)

        return b, b_dot, b_ddot
