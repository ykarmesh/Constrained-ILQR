
#########################################################################
#####                   Written By: Karmesh Yadav                   #####
#####                     Modified: 18/10/19                        #####
#########################################################################
import random
import time
import platform
import sys

# Display Python version
python_version = platform.python_version()
print('Python', python_version)
CARLA2 = '/mnt/data/Builds/CARLA_0.9.6/PythonAPI/carla/dist/carla-0.9.5-py2.7-linux-x86_64.egg'
if python_version.startswith('3'):
    if CARLA2 in sys.path: sys.path.remove(CARLA2)

import carla
from agents.navigation.basic_agent import BasicAgent
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO
from agents.navigation.local_planner import RoadOption
from route_manipulation import interpolate_trajectory

import arguments

DEBUG = True

class Carla_Interface():
    def __init__(self, town='Town01'):
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

        self.spawn_ego_vehicle()

        self.vehicles_list = []
        self.plan = []
        self.navigation_agent = None
        self.create_global_plan()

    def spawn_ego_vehicle(self):
        # Get a random vehicle from world
        blueprint = random.choice(self.world.get_blueprint_library().filter('bmw'))
        blueprint.set_attribute('role_name', 'hero')

        # Set spawn point(start) and goal point according to use case
        self.spawn_point = random.choice(self.spawn_points)
        print("Spawned vehicle at position: {}".format(self.spawn_point.location))
        self.ego_vehicle = self.world.try_spawn_actor(blueprint, self.spawn_point)

    def create_global_plan(self, end_point=None):
        if end_point == None:
            end_point = random.choice(self.spawn_points)
        print("Goal position: {}".format(end_point.location))

        # Give set of intermediate points spaced far away from start to goal
        rough_route = [self.spawn_point.location, end_point.location]

        # Interpolate the route to waypoints which are spaced 1m apart
        _, route = interpolate_trajectory(self.world, rough_route)

        # Make a Plan list which stores points as desired by BasicAgent Class
        for transform, road_option in route:
            wp = world_map.get_waypoint(transform.location)
            self.plan.append((wp, road_option))

        return self.plan

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
        self.navigation_agent = BasicAgent(ego_vehicle, target_speed)
        self.navigation_agent._local_planner.set_global_plan(self.plan)

    def run_step_pid(self):
        # Loop for controls
        assert self.navigation_agent != None, "Navigation Agent not initialized"

        while True:
            control = self.navigation_agent.run_step(debug=DEBUG)
            ego_vehicle.apply_control(control)
            if DEBUG:
                print(ego_vehicle.get_location())

    def spawn_npc(self, number_of_vehicles=10):
        blueprints = self.world.get_blueprint_library().filter('vehicle.*')
        blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
        blueprints = [x for x in blueprints if not x.id.endswith('isetta')]
        blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]

        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor

        batch = []
        for n, transform in enumerate(self.spawn_point):
            if n >= number_of_vehicles:
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
        
        for response in client.apply_batch_sync(batch):
            if response.error:
                logging.error(response.error)
            else:
                vehicles_list.append(response.actor_id)

    def get_npc_state(self):
        vehicle_states = []
        for n in range(self.vehicles_list):
            vehicle_transform = vehicles_list[n].get_transform()
            vehicle_velocity = vehicles_list[n].get_velocity()
            vehicle_angular_velocity = vehicles_list[n].get_angular_velocity()

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
        for n in range(self.vehicles_list):
            bbs.append(np.append([2*vehicles_list[n].bounding_box.x,
                                  2*vehicles_list[n].bounding_box.y]))

    def destroy(self):
        print('\ndestorying ego vehicle')
        self.ego_vehicle.destroy()

        print('\ndestroying %d vehicles' % len(vehicles_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])


if __name__ == "__main__":
    try:
        carla_interface = Carla_Interface()
        carla_interface.create_pid_agent()
        carla_interface.run_step_pid()
    except KeyboardInterrupt:
        pass
    finally:
        carla_interface.destroy()
        print('\ndone')
