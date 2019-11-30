import math
import numpy as np 
import matplotlib.pyplot as plt
import scipy.integrate as integrate
import pdb

from ilqr.vehicle_model import Model
from ilqr.local_planner import LocalPlanner
from ilqr.constraints import Constraints


class iLQR():
    def __init__(self, args, obstacle_bb, verbose=False):
        self.args = args
        self.Ts = args.timestep
        self.N = args.horizon
        self.tol = args.tol
        self.obstacle_bb = obstacle_bb
        self.verbose = verbose
        
        self.global_plan = None
        self.local_planner = LocalPlanner(args)
        self.vehicle_model = Model(args)
        self.constraints = Constraints(args)
        
        # initial nominal trajectory
        self.control_seq = np.zeros((self.args.num_ctrls, self.args.horizon))
        self.control_seq[0, :] = np.ones((self.args.horizon)) * 0.1

        self.fig, self.ax = plt.subplots()

    
    def set_global_plan(self, global_plan):
        self.global_plan = global_plan
        self.local_planner.set_global_planner(self.global_plan)

    def get_nominal_trajectory(self, X_0, U):
        X = np.zeros((self.args.num_states, self.args.horizon))
        X[:, 0] = X_0
        for i in range(self.args.horizon-1):
            X[:, i+1] = self.vehicle_model.forward_simulate(X[:, i], U[:, i])
        return X

    def forward_pass(self, X, U, k, K):
        X_new = np.zeros((self.args.num_states, self.args.horizon))
        X_new[:, 0] = X[:, 0]
        U_new = np.zeros((self.args.num_ctrls, self.args.horizon))
        # Do a forward rollout and get states at all control points
        for i in range(self.args.horizon-1):
            U_new[:, i] = U[:, i] + k[:, i] + K[:, :, i] @ (X_new[:, i] - X[:, i])
            X_new[:, i+1] = self.vehicle_model.forward_simulate(X_new[:, i], U_new[:, i])
        return X_new, U_new

    def backward_pass(self, X, U, poly_coeff, x_local_plan):
        # Find control sequence that minimizes Q-value function
        # Get derivatives of Q-function wrt to state and control
        l_x, l_xx, l_u, l_uu, l_ux = self.constraints.get_cost_derivatives(X, U, poly_coeff, x_local_plan) 
        df_dx = self.vehicle_model.get_A_matrix(X[2,:], X[3,:], U[0,:])
        df_du = self.vehicle_model.get_B_matrix(X[3,:])
        # Value function at final timestep is known
        V_x = l_x[:,-1] 
        V_xx = l_xx[:,:,-1]
        # Allocate space for feedforward and feeback term
        k = np.zeros((self.args.num_ctrls, self.args.horizon))
        K = np.zeros((self.args.num_ctrls, self.args.num_states, self.args.horizon))
        # Run a backwards pass from N-1 control step
        for i in range(self.args.horizon-1,-1,-1):
            Q_x = l_x[:,i] + df_dx[:,:,i].T @ V_x
            Q_u = l_u[:,i] + df_du[:,:,i].T @ V_x
            Q_xx = l_xx[:,:,i] + df_dx[:,:,i].T @ V_xx @ df_dx[:,:,i] 
            Q_ux = l_ux[:,:,i] + df_du[:,:,i].T @ V_xx @ df_dx[:,:,i]
            Q_uu = l_uu[:,:,i] + df_du[:,:,i].T @ V_xx @ df_du[:,:,i]
            Q_uu_inv = np.linalg.pinv(Q_uu)
            # Calculate feedforward and feedback terms
            k[:,i] = -Q_uu_inv @ Q_u
            K[:,:,i] = -Q_uu_inv @ Q_ux
            # Update value function for next time step
            V_x = Q_x - K[:,:,i].T @ Q_uu @ k[:,i]
            V_xx = Q_xx - K[:,:,i].T @ Q_uu @ K[:,:,i]
            # V_x  = Q_x + Q_ux.T @ k[:,i] + K[:,:,i].T @ Q_u + K[:, :, i).T @ Q_uu @ k[:, i] # Sergey
            # V_xx = Q_xx + K[:,:,i].T @ Q_ux + Q_ux.T @ K[:,:,i] + K[:, :, i).T @ Q_uu @ K[:, :, i] # Sergey
        
        return k, K


    def run_step(self, ego_state, npc_states):
        assert self.global_plan is not None, "Set a global plan in iLQR before starting run_step"

        self.local_planner.set_ego_state(ego_state)
        ref_traj, poly_coeff = self.local_planner.get_local_plan()

        X_0 = np.array([ego_state[0][0], ego_state[0][1], ego_state[1][0], ego_state[2][2]])
        
        U = self.get_optimal_control_seq(X_0, self.control_seq, poly_coeff, ref_traj[:, 0])
        self.control_seq = U
        self.plot(U)
        return ref_traj, self.filter_control(U[:, 0],  ego_state[1][0])

    def get_optimal_control_seq(self, X_0, U, poly_coeff, x_local_plan):
        # pdb.set_trace()
        X = self.get_nominal_trajectory(X_0, U)
        # Run iLQR for max iterations
        for itr in range(self.args.max_iters):
            k, K = self.backward_pass(X, U, poly_coeff, x_local_plan)
            # Get control values at control points and new states
            # again by a forward rollout
            X_new, U_new = self.forward_pass(X, U, k, K)
        
        return U_new

    def filter_control(self, U, velocity):
        U[1] = math.atan2(self.args.wheelbase*U[1],velocity)
        return U

    def plot(self, control):
        self.ax.clear()
        self.ax.plot(np.arange(len(control[0])), control[0,:], color='g', label='Acc')
        self.ax.plot(np.arange(len(control[0])), control[1,:], color='b', label='Yaw Rate')
        plt.legend()
        plt.pause(0.001)
        


