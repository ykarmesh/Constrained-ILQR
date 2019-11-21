import numpy as np

class LocalPlanner:
    """
    Class which creates a desired trajectory based on the global plan
    Created by iLQR class based on the plan provided by the simulator
    """
    def __init__(self, args):
        self.args = args
        self.global_plan = None
        self.state = None
        # self.control = control
    
    def set_current_state(self, ego_state):
        self.ego_state = ego_state

    def set_global_planner(self, global_plan):
        """
        Sets the global plan of the ego vehicle
        """
        self.global_plan = global_plan

    def closest_node(self, node, nodes):
        nodes = np.asarray(nodes)
        closest_ind = np.sum((nodes - node)**2, axis=1)
        return np.argmin(closest_ind)

    def get_local_plan_wpts(self, global_wpts):
        """
        Creates a local plan based on the waypoints on the global planner 
        """
        assert ego_state is not None, "Ego state was not set in the LocalPlanner"

        # Find index of waypoint closest to current pose
        closest_ind = self.closest_node([self.state[0],self.state[1]], global_wpts) 
        local_wpts = [[global_wpts[i,0],global_wpts[i,1]] for i in range(closest_ind, closest_ind + self.args.number_of_local_wpts)]
        return local_wpts

    def local_plan_function(self, global_wpts):
        local_wpts = get_local_plan_wpts(global_wpts)
        x = local_wpts[:,0]
        y = local_wpts[:,1]
        coeffs = np.polyfit(x, y, 3)
        return coeffs

    def get_orientation(self):
        """
        Gets the orientation of the path at the closest point on the local plan to be used by iLQR
        """
        raise NotImplementedError
    