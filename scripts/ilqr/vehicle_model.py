import numpy as np

class Model:
    def __init__(self, args):
        self.wheelbase = args.wheelbase
        self.steer_min = args.steering_limits[0]
        self.steer_max = args.steering_limits[1]
        self.accel_min = args.acceleration_limits[0]
        self.accel_max = args.acceleration_limits[1]
        self.Ts = args.timestep
        self.N = args.horizon
        
    def forward_simulate(self, state, control):
        next_state = np.array([[state[0] + cos(state[3])*(state[2]*self.Ts + (control[0]*Ts**2)/2)],
                               [state[1] + sin(state[3])*(state[2]*self.Ts + (control[1]*Ts**2)/2)],
                               [state[2] + control[0]*self.Ts],
                               [state[3] + control[1]*self.Ts]])
        return next_state

    def get_A_matrix(self, theta, velocity, acceleration):
        """
        Returns the linearized 'A' matrix of the ego vehicle 
        model for all states in backward pass. 
        """
        v = velocity
        v_dot = acceleration
        z = np.zeros((self.N))
        A = np.array([[z, z, cos(theta), -(v + v_dot*self.Ts)*sin(theta)],
                      [z, z, sin(theta),  (v + v_dot*self.Ts)*cos(theta)],
                      [z, z,          z,                               z],
                      [z, z,          z,                               z]])
        return A

    def get_B_matrix(self, theta):
        z = np.zeros((self.N))
        o = np.ones((self.N))
        B = np.array([[self.Ts*cos(theta), z],
                      [self.Ts*cos(theta), z],
                      [                 o, z],
                      [                 z, o]])
        return B
