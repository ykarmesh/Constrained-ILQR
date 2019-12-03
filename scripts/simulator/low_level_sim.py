import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches
from sklearn.preprocessing import normalize
from matplotlib import animation
import math

PI = math.pi

class PolyRect:

    def __init__(self, dim):
        self.dimension = dim

        self.corners = np.zeros((4,2))
        self.axes = np.zeros((2,2))

        self.R = np.zeros((2,2))

    # Finds the corners and axes 
    def createCuboid(self, state):
        self.origin = state[0:2]
        self.orientation = state[2]

        # Creates Rotation Matrix for cuboid 
        self.R = np.array([[math.cos(self.orientation), -math.sin(self.orientation)],
                           [math.sin(self.orientation),  math.cos(self.orientation)]])

        self.findCorners()

        self.findAxes()        


    def findCorners(self):
        # Corners of a cuboid of length one and orientation of zero along x,y and z
        direction = np.array([[ 0.5, 0.5],[-0.5, 0.5], \
                              [ -0.5,-0.5],[0.5,-0.5]])

        # Dimension along x,y and z
        D = np.tile(self.dimension, (4, 1))

        # Cuboid scaled according to dimensions
        direction = direction*D

        # Origin of the cuboid
        O = np.tile(self.origin, (4,1))

        # Corners after rotation by R and translation by O
        self.corners = np.matmul(self.R, (direction).T).T + O

    def findAxes(self):
        # Axis of the cuboid before rotation
        direction = np.array([[1,0],[0,1]])

        # Rotation and normalization
        self.axes = np.matmul(self.R, direction).T
        self.axes = normalize(self.axes, axis=1)

    
    def getPatch(self, ax, color='r'):
        # plt.close()
        # self.corners[[2,3]] = self.corners[[3,2]]
        rect = matplotlib.patches.Polygon(self.corners[:,:], color=color, alpha=0.50)

        # angle = np.pi*self.orientation/180.0
        # t2 = matplotlib.transforms.Affine2D().rotate_deg(angle) + ax.transData
        # rect.set_transform(t2)

        return rect

    def draw(self, ax, color='r'):
        # plt.close()
        # self.corners[[2,3]] = self.corners[[3,2]]
        rect = matplotlib.patches.Polygon(self.corners[:,:], color=color, alpha=0.50)

        # angle = np.pi*self.orientation/180.0
        # t2 = matplotlib.transforms.Affine2D().rotate_deg(angle) + ax.transData
        # rect.set_transform(t2)

        ax.add_patch(rect)
    
    def getCorners(self):
        return self.corners




##
total_states = [np.array([10.0, 0.0, 0]),np.array([15.0, 0.0, PI/6]),np.array([20.0, 0.0, PI/3])]
fig = plt.figure(figsize=(25,5))
# plt.axis('equal')
ax = fig.add_subplot(111)
ax.axis('equal')
ax.set_xlim(0, 50)
ax.set_ylim(-50, 50)
ax.axhline(y=3)

ref_cube = [np.array([0.0, 0.0, 0.0]), np.array([2, 2])] # state and dimension
ego_dims = np.array([2, 1])
ego_init = np.array([0.0, 0.0, 0.0])


ego_cube = PolyRect(ego_dims) # instantiate ego car
# ego_cube.createCuboid(total_states[0])
# print(ego_cube.getCorners())
ego_patch = ego_cube.getPatch(ax)

ax.add_patch(ego_patch)

def init():
    return ego_patch,

def animate(i):
    # Get new corners of cuboid
    curr_state = total_states[i]
    ego_cube.createCuboid(curr_state)
    new_corner = ego_cube.getCorners()
    # new_corner =  np.array([[3+i, 0], [5+i, 0.], [5, 2+i], [3, 2+i]])
    ego_patch.set_xy(new_corner)
    # Set patch corners to new corners
    return ego_patch,

anim = animation.FuncAnimation(fig, animate,
                               init_func=init,
                               frames=len(total_states),
                               interval=1000,
                               blit=True)
plt.show()


##

# patch = patches.PolyRect((0, 0), 0, 0, fc='y')

# x = [0, 1, 2, 3, 4, 5]
# y = [0, 0, 0, 0, 0, 0]
# yaw = [10, 30, 40, 50, 60, 90]
# fig = plt.figure()
# plt.axis('equal')
# # plt.grid()
# ax = fig.add_subplot(111)
# ax.set_xlim(0, 10)
# ax.set_ylim(-10, 10)
# def init():
#     ax.add_patch(patch)
#     return patch,

# def animate(i):
#     patch.set_width(2)
#     patch.set_height(1.0)
#     patch.set_xy([x[i], y[i]])
#     patch._angle = -np.rad2deg(yaw[i])
#     return patch,

# anim = animation.FuncAnimation(fig, animate,
#                                init_func=init,
#                                frames=len(x),
#                                interval=1000,
#                                blit=True)
# plt.show()