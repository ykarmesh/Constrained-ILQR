import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches
from sklearn.preprocessing import normalize
from matplotlib import animation
import math
from PolyRect import PolyRect
PI = math.pi

class PySimulator:

    def __init__(self, num_vehicles, NPC_trajs):
        # num_vehicles is only 2 for now
        self.num_vehicles = num_vehicles
        self.NPC_dict = {}
        self.patches = []
        car_dims = np.array([2, 1])

        # Plot parameters
        self.fig = figure(figsize=(25,5))
        self.ax = fig.add_subplot(111)
        self.ax.axis('equal')
        self.ax.set_xlim(0, 50)
        self.ax.set_ylim(-50, 50)
        self.ax.axhline(y=4, c='k', lw='4')
        self.ax.axhline(y=0, c='k', lw='2', ls='--')
        self.ax.axhline(y=-4, c='k', lw='4')

        # Ego vehicle is first
        for i in range(num_vehicles):
            self.NPC_dict[i] = PolyRect(car_dims)
            self.patches.append(self.NPC_dict[i].getPatch(self.ax))
        
        # May need to add loop here
        self.ax.add_patch(self.patches[0])
        self.ax.add_patch(self.patches[1])

    def init(self):
        return self.patches[0], self.patches[1],
    
    def run_iLQR(self):
        # Run iLQR to get control sequence
    
    def animate(self):
        # Get new corners of cuboid
        # Set patch corners to new corners

        return self.patches[0], self.patches[1],

    def run_simulation(self):
        anim = animation.FuncAnimation(fig, animate,
                               init_func=init,
                               frames=len(total_states),
                               interval=1000,
                               blit=True)
        plt.show()


