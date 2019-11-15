import numpy as np

class LocalPlanner:
    """
    Class which creates a desired trajectory based on the global plan
    Created by iLQR class based on the plan provided by the simulator
    """
    def __init__(self, args):
        self.args = args
        self.global_plan = None

    def set_global_planner(self, global_plan):
        """
        Sets the global plan of the ego vehicle
        """
        self.global_plan = global_plan

    def create_local_plan_function(self):
        """
        Creates a local plan based on the waypoints on the global planner 
        """
        local_wpts = [[wpts[i,0],wpts[i,1]] for i in range(self.args.number_of_local_wpts)]
        x = local_wpts[:,0]
        y = local_wpts[:,1]
        coeffs = np.polyfit(x, y, 3)
        return coeffs
        
    def get_local_plan(self)
        coeffs = self.create_local_plan_function()
        # Change dependinng on discretization
        local_plan = [[x[i],x[i]**3*coeffs[0] + x[i]**2*coeffs[1] + x[i]*coeffs[2] + coeffs[3]] for i in range(self.args.number_of_local_wpts)]

        return local_plan

    def get_orientation(self):
        """
        Gets the orientation of the path at the closest point on the local plan to be used by iLQR
        """
        raise NotImplementedError
    