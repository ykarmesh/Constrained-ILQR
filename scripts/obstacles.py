import numpy as np 

class Obstacle:
    def __init__(self, track_id, car_length, car_width):
        self.car_length = car_length
        self.car_width = car_width
        self.track_id = track_id

    def get_obstacle_cost(self, pose):
        pass

    