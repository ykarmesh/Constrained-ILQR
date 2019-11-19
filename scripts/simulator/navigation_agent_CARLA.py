
#########################################################################
#####                   Written By: Karmesh Yadav                   #####
#####                     Modified: 18/10/19                        #####
#########################################################################
import argparse
import logging
import os
import platform
import pdb
import random
import sys
import time

import numpy as np
import pygame

# Display Python version
python_version = platform.python_version()
print('Python', python_version)
CARLA2 = '/mnt/data/Builds/CARLA_0.9.6/PythonAPI/carla/dist/carla-0.9.6-py2.7-linux-x86_64.egg'
if python_version.startswith('3'):
    if CARLA2 in sys.path: 
        sys.path.remove(CARLA2)
        print("Deleting Carla 2.7 from pythonpath")

try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
except IndexError:
    pass

import carla
from agents.navigation.basic_agent import BasicAgent
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO
from agents.navigation.local_planner import RoadOption
from route_manipulation import interpolate_trajectory

from arguments import add_arguments
from scripts.iLQR import iLQR

class Carla_Interface():
    def __init__(self, args, town='Town01', verbose=False):
        self.args = args
        self.verbose = verbose
        self.client = carla.Client('127.0.0.1', 2000)
        self.client.set_timeout(10.0)

        # Load Desired Map
        if (self.client.get_world().get_map().name != town):
            self.client.load_world(town)
            time.sleep(10.0)
    
        # Get world and map from carla
        self.world = self.client.get_world()
        self.world_map = self.world.get_map()

        # Find valid points for spawning the vehicle
        self.spawn_points = self.world_map.get_spawn_points()

        self.sensors = {}
        self.vehicles_list = []
        self.plan_pid = []
        self.plan_ilqr = []
        self.navigation_agent = None

        self.spawn_ego_vehicle()
        if self.args.add_npc_agents:
            self.spawn_npc()

        self.create_global_plan()
        self.setup_rendering()

    def setup_rendering(self):
        pygame.init()

        w = self.args.camera_width
        h = self.args.camera_height
        
        self.display = pygame.display.set_mode((2*w, h), pygame.HWSURFACE | pygame.DOUBLEBUF)

        self.camera_1 = pygame.Rect(0,0,w,h)
        self.camera_2 = pygame.Rect(w,0,2*w,h)

        self.surface = {}
        # Setup Camera 2
        cam_transform = carla.Transform(carla.Location(x=1, y=0, z=30.0), carla.Rotation(pitch=-90, roll=0 ,yaw=0))
        self.add_sensor('rgb_top', w, h, cam_transform, self.image_callback)

        # Setup Camera 1
        cam_transform = carla.Transform(carla.Location(x=1, y=0, z=2.0), carla.Rotation(pitch=0, roll=0 ,yaw=0))
        self.add_sensor('rgb_front', w, h, cam_transform, self.image_callback)

    def spawn_ego_vehicle(self):
        # Get a random vehicle from world
        blueprint = random.choice(self.world.get_blueprint_library().filter('tesla'))
        blueprint.set_attribute('role_name', 'hero')

        # Set spawn point(start) and goal point according to use case
        self.spawn_point = random.choice(self.spawn_points)
        print("\nSpawned vehicle at position: {}".format(self.spawn_point.location))
        self.ego_vehicle = self.world.try_spawn_actor(blueprint, self.spawn_point)

    def add_sensor(self, sensor_tag, image_width, image_height, transform, callback):
        bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        bp.set_attribute('image_size_x', str(image_width))
        bp.set_attribute('image_size_y', str(image_height))
        bp.set_attribute('fov', str(100))
        bp.set_attribute('sensor_tick', str(0.0333))

        self.sensors[sensor_tag] = self.world.spawn_actor(bp, transform, self.ego_vehicle)
        self.sensors[sensor_tag].listen(lambda image: callback(sensor_tag, image))
        # self.sensors[sensor_tag].listen(lambda image: image.save_to_disk('output/%06d.png' % image.frame))

    def image_callback(self, sensor_tag, image):
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self.surface[sensor_tag] = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        
        if(sensor_tag == 'rgb_front'):
            self.display.blit(self.surface[sensor_tag], self.camera_1)
        else:
            self.display.blit(self.surface[sensor_tag], self.camera_2)

    def create_global_plan(self, end_point=None):
        if end_point == None:
            end_point = random.choice(self.spawn_points)
        print("\nGoal position: {}".format(end_point.location))

        # Give set of intermediate points spaced far away from start to goal
        rough_route = [self.spawn_point.location, end_point.location]

        # Interpolate the route to waypoints which are spaced 1m apart
        _, route = interpolate_trajectory(self.world, rough_route)

        # Make a Plan list which stores points as desired by BasicAgent Class
        for transform, road_option in route:
            wp = self.world_map.get_waypoint(transform.location)
            self.plan_pid.append((wp, road_option))
            self.plan_ilqr.append(np.array([wp.transform.location.x, wp.transform.location.y]))


    def get_ego_states(self):
        vehicle_transform = self.ego_vehicle.get_transform()
        vehicle_velocity = self.ego_vehicle.get_velocity()
        vehicle_angular_velocity = self.ego_vehicle.get_angular_velocity()

        ego_states = np.array([vehicle_transform.location.x,
                               vehicle_transform.location.y,
                               vehicle_velocity.x,
                               vehicle_velocity.y,
                               vehicle_transform.rotation.yaw,
                               vehicle_angular_velocity.z])
        
        return ego_states

    def create_pid_agent(self, target_speed=20):
        # Carla Funtion for Creating a Basic Navigation Agent
        self.navigation_agent = BasicAgent(self.ego_vehicle, target_speed)
        self.navigation_agent._local_planner.set_global_plan(self.plan_pid)

    def run_step_pid(self):
        # Loop for controls
        assert self.navigation_agent != None, "Navigation Agent not initialized"

        while True:
            control = self.navigation_agent.run_step(debug=self.verbose)
            self.ego_vehicle.apply_control(control)

            # render scene
            pygame.display.flip()

            if self.verbose:
                print(self.ego_vehicle.get_location())

    def create_ilqr_agent(self):
        self.navigation_agent = iLQR(self.args, self.get_npc_bounding_box())
        self.navigation_agent.set_global_plan(self.plan_ilqr)

    def run_step_ilqr(self):
        assert self.navigation_agent != None, "Navigation Agent not initialized"

        while True:
            control = self.navigation_agent.run_step()
            self.ego_vehicle.apply_control(control)


    def spawn_npc(self):
        blueprints = self.world.get_blueprint_library().filter('vehicle.*')
        blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
        blueprints = [x for x in blueprints if not x.id.endswith('isetta')]
        blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]

        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor

        batch = []
        for n, transform in enumerate(self.spawn_points):
            if n >= self.args.number_of_npc:
                break
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            blueprint.set_attribute('role_name', 'autopilot')
            batch.append(SpawnActor(blueprint, transform).then(SetAutopilot(FutureActor, True)))
        
        id_list = []
        for response in self.client.apply_batch_sync(batch):
            if response.error:
                logging.error(response.error)
            else:
                id_list.append(response.actor_id)
        self.vehicles_list = self.world.get_actors(id_list)

    def get_npc_state(self):
        vehicle_states = []
        for n in range(len(self.vehicles_list)):
            vehicle_transform = self.vehicles_list[n].get_transform()
            vehicle_velocity = self.vehicles_list[n].get_velocity()
            vehicle_angular_velocity = self.vehicles_list[n].get_angular_velocity()

            vehicle_states.append(np.array([vehicle_transform.location.x,
                                            vehicle_transform.location.y,
                                            vehicle_velocity.x,
                                            vehicle_velocity.y,
                                            vehicle_transform.rotation.yaw,
                                            vehicle_angular_velocity.z]))
        return vehicle_states

    def get_npc_bounding_box(self):
        """
        Gives the x and y dimension of the bounding box
        """
        bbs = []
        for n in range(len(self.vehicles_list)):
            bbs.append(np.array([2*self.vehicles_list[n].bounding_box.extent.x,
                                 2*self.vehicles_list[n].bounding_box.extent.y]))
        return bbs

    def destroy(self):
        print('\ndestorying ego vehicle')
        self.ego_vehicle.destroy()

        print('\ndestroying %d vehicles' % len(self.vehicles_list))
        self.client.apply_batch([carla.command.DestroyActor(x.id) for x in self.vehicles_list])

        print('\ndestroying %d sensors' % len(self.sensors))        
        for sensor in self.sensors.values():
            sensor.destroy()

        pygame.quit()


if __name__ == "__main__":
    try:
        argparser = argparse.ArgumentParser(description='CARLA CILQR')
        add_arguments(argparser)
        args = argparser.parse_args()
        pdb.set_trace()
        carla_interface = Carla_Interface(args)
        carla_interface.create_pid_agent()
        carla_interface.run_step_pid()
    except KeyboardInterrupt:
        pass
    finally:
        carla_interface.destroy()
        print('\ndone')
