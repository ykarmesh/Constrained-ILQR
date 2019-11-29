import numpy as np 
import math
from scipy.optimize import fmin_cobyla

class Constraints:
	def __init__(self, args):
		self.args = args
		self.control_cost = np.array([[self.args.w_acc,                   0],
									  [              0, self.args.w_yawrate]])

		self.state_cost = np.array([[self.args.w_pos, 0, 0, 0],
									[0, self.args.w_pos, 0, 0],
									[0, 0, self.args.w_vel, 0],
									[0, 0, 0,               0]])
		self.coeffs = None

	

	def get_control_cost_derivatives(self, state, control):
		"""
		Returns the control quadratic (R matrix) and linear cost term (r vector) for the trajectory
		"""
		P1 = np.array([[1],[0]])
		P2 = np.array([[0],[1]])

		l_u = np.zeros((self.args.num_ctrls, self.args.horizon))
		l_uu = np.zeros((self.args.num_ctrls, self.args.num_ctrls, self.args.horizon))

		for i in range(len(self.args.horizon)):
			# Acceleration Barrier Max
			c = (np.matmul(control[i].T, P1) - self.args.acc_limits[1])
			b_1, b_dot_1, b_ddot_1 = self.barrier_function(self.args.q1_acc, self.args.q2_acc, c, P1)

			# Acceleration Barrier Min
			c = (self.args.acc_limits[0] - np.matmul(control[i].T, P1))
			b_2, b_dot_2, b_ddot_2 = self.barrier_function(self.args.q1_acc, self.args.q2_acc, c, -P1)

			# Yawrate Barrier Max
			c = (np.matmul(control[i].T, P2) - state[i,2]*math.tan(self.args.steer_angle_limits[1])/self.args.wheelbase)
			b_3, b_dot_3, b_ddot_3 = self.barrier_function(self.args.q1_yawrate, self.args.q2_yawrate, c, P2)

			# Yawrate Barrier Min
			c = (state[i,2]*math.tan(self.args.steer_angle_limits[0])/self.args.wheelbase - np.matmul(control[i].T, P2))
			b_4, b_dot_4, b_ddot_4 = self.barrier_function(self.args.q1_yawrate, self.args.q2_yawrate, c, -P2)

			l_u_i = b_dot_1 + b_dot_2 + b_dot_3 + b_dot_4
			l_uu_i = b_ddot_1 + b_ddot_2 + b_ddot_3 + b_ddot_4 + self.control_cost

			l_u[:, i] = l_u_i
			l_uu[:, :, i] = l_uu_i

		return l_u, l_uu

	def barrier_function(self, q1, q2, c, c_dot):
		b = q1*np.exp(q2*c)
		b_dot = q1*q2*np.exp(q2*c)*c_dot
		b_ddot = q1*(q2**2)*np.exp(q2*c)*np.matmul(c_dot, c_dot.T)

		return b, b_dot, b_ddot

	def get_cost_derivatives(self, state, control):
		"""
		Returns the different cost terms for the trajectory
		This is the main function which calls all the other functions 
		"""
		self.coeffs = poly_coeffs

		l_u, l_uu = self.get_control_cost_derivatives(state, control)
		l_x, l_xx = self.get_state_cost_derivatives(state)

		return l_x, l_xx, l_u, l_uu

	def get_state_cost_derivatives(self, state):
		"""
		Returns the first order and second order derivative of the value function wrt state
		"""
		l_x = np.zeros((self.args.num_states, self.args.horizon))
		l_xx = np.zeros((self.args.num_states, self.args.num_states, self.args.horizon))

		for i in range(len(self.args.horizon)):
			# Offset in path derivative
			X = fmin_cobyla(offset_obj, x0=[state[0], state[1]], cons=[c1])
			x_r, y_r = X
			dc_off = self.args.w_pos*2*np.array([state[0]-x_r, state[1]-y_r, 0, 0])
			# Offset in velocity derivative 
			dc_vel = self.args.w_vel*2*np.array([0,0,state[2]-self.args.desired_speed,0])
			# Compute first order derivative TODO: add obstacles constraints
			dl_dx = dc_off + dc_vel 
			# Compute second order derivative
			l_xx_i = self.state_cost #TODO: add obstacles constraints

			l_xx[:, :, i] = l_xx_i

		return l_x, l_xx

	"""
	Unused stuff
	"""
	# def get_acceleration_cost(self): 
	# 	return np.matmul(np.matmul(self.args.w_acc*self.control.T*np.array([[1,0],[0,0]]))*self.control)

	# def get_yawrate_cost(self):
	# 	return np.mamtul(np.matmul(self.args.w_acc*self.control.T*np.array([[0,0],[0,1]]))*self.control)

	def desired_pose_function(self, x):
		return np.polyval(self.coeffs,x)

	def offset_obj(self, X):
		x,y = X
		return np.sqrt((x - self.state[0])**2 + (y - self.state[1])**2)

	def c1(self, X):
		x,y = X
		return self.desired_pose_function(x) - y

	def get_offset_cost(self):
		# Get closest point from the curve
		X = fmin_cobyla(offset_obj, x0=[self.state[0],self.state[1]], cons=[c1])
		x_r, y_r = X
		state_diff = np.array([state[0]-x_r, state[1]-y_r])
		Qk = np.array([[1,0,0],[0,1,0],[0,0,self.args.w_vel]])

		return np.matmul(np.matmul(state_diff.T*Q),state_diff)

	def get_velocity_cost(self):
		return self.args.w_vel*(abs(self.state[2]) - self.desired_speed)

