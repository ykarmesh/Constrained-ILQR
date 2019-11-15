import numpy as np 
import math
from scipy.optimize import fmin_cobyla

class Constraints:
    def __init__(self, args, state, control, poly_coeffs, desired_speed):
        self.args = args
        self.control_cost = np.array([[self.args.w_acc,                   0],
                                      [              0, self.args.w_yawrate]])

        self.state_cost = np.array([[self.args.w_pos, 0, 0, 0],
                                    [0, self.args.w_pos, 0, 0],
                                    [0, 0, self.args.w_vel, 0],
                                    [0, 0, 0,               0]])
        self.state = state
        self.control = control
        self.coeffs = poly_coeffs
        self.desired_speed = desired_speed
    
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

    def get_acceleration_cost(self): 
    	return np.matmul(np.matmul(self.args.w_acc*self.control.T*np.array([[1,0],[0,0]]))*self.control)

	def get_yawrate_cost(self):
		return np.mamtul(np.matmul(self.args.w_acc*self.contro.T*np.array([[0,0],[0,1]]))*self.control)

	def desired_pose_function(self, x):
        return x**3*self.coeffs[0] + x**2*self.coeffs[1] + x*self.coeffs[2] + self.coeffs[3]

    def offset_obj(self, X):
		x,y = X
	 	return np.sqrt((x - self.state[0])**2 + (y - self.state[1])**2)

	def c1(X):
		x,y = X
	    return desired_pose_function(x) - y

	def get_offset_cost(self):
		# Get closest point from the curve
		X = fmin_cobyla(offset_obj, x0=[self.state[0],self.state[1]], cons=[c1])
		x_r, y_r = X
		state_diff = np.array([state[0]-x_r, state[1]-y_r])
		Qk = np.array([[1,0,0],[0,1,0],[0,0,self.args.w_vel]])

		return np.matmul(np.matmul(state_diff.T*Q),state_diff)

	def get_velocity_cost(self):
		return self.args.w_vel*(abs(self.state[2]) - self.desired_speed)

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
