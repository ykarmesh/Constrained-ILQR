import numpy as np
# Add lambda functions
cos = lambda a : np.cos(a)
sin = lambda a : np.sin(a)

class Model:
    def __init__(self, args):
        self.wheelbase = args.wheelbase
        self.steer_min = args.steer_angle_limits[0]
        self.steer_max = args.steer_angle_limits[1]
        self.accel_min = args.acc_limits[0]
        self.accel_max = args.acc_limits[1]
        self.Ts = args.timestep
        self.N = args.horizon
        
    def forward_simulate(self, state, control):
        next_state = np.array([[state[0] + cos(state[3])*(state[2]*self.Ts + (control[0]*self.Ts**2)/2)],
                               [state[1] + sin(state[3])*(state[2]*self.Ts + (control[1]*self.Ts**2)/2)],
                               [state[2] + control[0]*self.Ts],
                               [state[3] + control[1]*self.Ts]])
        return next_state

    def get_A_matrix(self, velocity_vals, theta, acceleration_vals):
        """
        Returns the linearized 'A' matrix of the ego vehicle 
        model for all states in backward pass. 
        """
        z = np.zeros((self.N))
        o = np.ones((self.N))
        A = np.array([[o, z, cos(theta), -(v + v_dot*self.Ts)*sin(theta)],
                      [z, o, sin(theta),  (v + v_dot*self.Ts)*cos(theta)],
                      [z, z,          o,                               z],
                      [z, z,          z,                               o]])
        return A

    def get_B_matrix(self, theta):
        o = np.ones((self.N))
        B = np.array([[self.Ts**2*cos(theta), 0],
                      [self.Ts**2*cos(theta), 0],
                      [                 o, 0],
                      [                 0, o]])
        return B
