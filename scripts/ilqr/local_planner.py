import numpy as np

class LocalPlanner:
    """
    Class which creates a desired trajectory based on the global plan
    Created by iLQR class based on the plan provided by the simulator
    """
    def __init__(self, args):
        raise NotImplementedError

    def set_global_planner(self):
        """
        Sets the global plan of the ego vehicle
        """
        raise NotImplementedError

    def create_local_plan(self):
        """
        Creates a local plan based on the waypoints on the global planner 
        """
        raise NotImplementedError

    def get_closest_point(self):
        """
        Gets the closest point on the local plan to be used by iLQR
        """
        raise NotImplementedError

    def get_orientation(self):
        """
        Gets the orientation of the path at the closest point on the local plan to be used by iLQR
        """
        raise NotImplementedError
    