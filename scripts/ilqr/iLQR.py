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
        X = np.zeros((ctrl_pts, self.arg.num_states))
        X[0] = start_state
        # Do a forward rollout and get states at all control points
        for t in range(ctrl_pts-1):
            X[t+1] = self.vehicle_model.forward_simulate(X[t],control_seq[t])

        return X

    def backward_pass(self):
        # Find control sequence that minimizes Q-value function
        # Precompute derivatives of value function w.r.t state and control


    def run_step(self, ego_state, npc_states):
        assert self.global_plan is not None, "Set a global plan in iLQR before starting run_step"

        self.local_planner.set_ego_state(ego_state)
        path = self.local_planner.get_local_plan_waypoints()
        return path, 0.0

    def run_iteration(self):

    def get_optimal_control_seq(self, start_state, control_seq):
        X = self.forward_pass(start_state,control_seq)
        # Run iLQR for max iterations
        flag_itr = True # iteration stop flag
        for itr in range(self.args.max_iters):
            while flag_itr:
                # Get derivatives of Q-function wrt to state and control
                l_x, l_xx, l_u, l_ux, l_uu, f_x, f_u = get_Q_derivatives(X,control_seq,ref_traj)
                

