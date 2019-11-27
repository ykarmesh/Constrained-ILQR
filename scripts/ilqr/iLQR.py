import numpy as np 
import scipy.integrate as integrate

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
        # self.constraints = Constraints(args)
    
    def set_global_plan(self, global_plan):
        self.global_plan = global_plan
        self.local_planner.set_global_planner(self.global_plan)

    def forward_pass(self, start_state, control_seq):
        # ctrl_pts = self.args.number_of_local_wpts/self.args.timestep
        ctrl_pts = self.args.horizon # Maybe this
        X = np.zeros((self.args.num_states, ctrl_pts)) # State is 4 x ctrl_pts
        X[:,0] = start_state
        # Do a forward rollout and get states at all control points
        for t in range(ctrl_pts-1):
            X[:,t+1] = self.vehicle_model.forward_simulate(X[t],control_seq[t])

        return X

    def backward_pass(self):
        # Find control sequence that minimizes Q-value function
        # Get derivatives of Q-function wrt to state and control
        l_x, l_xx, l_u, l_ux, l_uu = get_Q_derivatives(X,control_seq,ref_traj) 
        df_dx = self.vehicle_model.get_A_matrix(X[2,:],X[3,:],control_seq[0,:])
        df_du = self.vehicle_model.get_B_matrix(X[3,:])
        # Value function at final timestep is known
        V_x = l_x[:,-1] 
        V_xx = l_xx[:,:,-1]
        # Allocate space for feedforward and feeback term
        k = np.zeros((self.args.num_ctrls,self.args.horizon))
        K = np.zeros((self.args.num_ctrls,self.args.num_states,self.args.horizon))
        # Run a backwards pass from N-1 control step
        for i in range(self.args.horizon-1,-1,-1):
            Q_x = l_x[:,i] + df_dx[:,:,i].T @ V_x
            Q_u = l_u[:,i] + df_du[:,:,i].T @ V_x
            Q_xx = l_xx[:,:,i] + df_dx[:,:,i].T @ V_xx@df_dx[:,:,i] 
            Q_ux = l_ux[:,:,i] + df_du[:,:,i].T @ V_xx@df_dx[:,:,i]
            Q_uu = l_uu[:,:,i] + df_du[:,:,i].T @ V_xx@df_du[:,:,i]
            Q_uu_inv = np.linalg.pinv(Q_uu)
            # Calculate feedforward and feedback terms
            k[:,i] = -Q_uu_inv @ Q_u
            K[:,:,i] = -Q_uu_inv @ Q_ux
            # Update value function for next time step
            V_x = Q_x - K[:,:,i].T @ Q_uu @ k[:,i]
            V_xx = Q_xx - K[:,:,i].T @ Q_uu @ K[:,:,i]
            # V_x  = Q_x + Q_ux.T @ k[:,i] + K[:,:,i].T @ Q_u + K[:, :, i).T @ Q_uu @ k[:, i] # Yanjun 
            # V_xx = Q_xx + K[:,:,i].T @ Q_ux + Q_ux.T @ K[:,:,i] + K[:, :, i).T @ Q_uu @ K[:, :, i] # Yanjun
        
        return k,K


            



    def run_step(self, ego_state, npc_states):
        assert self.global_plan is not None, "Set a global plan in iLQR before starting run_step"

        self.local_planner.set_ego_state(ego_state)
        path = self.local_planner.get_local_plan_waypoints()
        return path, 0.0

    def run_iteration(self):

    def get_optimal_control_seq(self, start_state, control_seq):
        X = self.forward_pass(start_state,control_seq)
        # Run iLQR for max iterations
        for itr in range(self.args.max_iters):
            k, K = self.backward_pass()
            
                


