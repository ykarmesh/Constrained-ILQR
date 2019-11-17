import sys
import numpy as np

import carla
from simple_pid import PID

from simulator.carla_control_physics import VehiclePhysicsInfo


class LowLevelController():
    def __init__(self, carla_vehicle_info, verbose=False):
        self.carla_vehicle_info = carla_vehicle_info
        self.carphysics = VehiclePhysicsInfo(self.carla_vehicle_info)
        self.verbose = verbose

    def get_control(self, vehicle_state, accel, steering_angle):
        self.vehicle_state = vehicle_state
        self.reverse = vehicle_state[2] < 0

        control = carla.VehicleControl()
        
        control.throttle, control.brake = get_throttle_brake_control(accel)
        control.steer = self.get_steering_control(steering_angle)
                
        # finally clip the final control output (should actually never happen)
        control.brake = np.clip(control.brake, 0., 1.)
        control.throttle = np.clip(control.throttle, 0., 1.)
    
        return control

    def set_target_steering_angle(self, target_steering_angle):
        """
        set target sterring angle
        """
        steering_angle = np.clip(target_steering_angle, -self.carphysics.max_steering_angle, self.carphysics.max_steering_angle)

        if abs(steering_angle) > self.carphysics.max_steering_angle and self.verbose:
            print("Max steering angle reached, clipping value")

        return steering_angle

    def set_target_speed(self, target_speed):
        """
        set target speed
        """
        speed = np.clip(target_speed, -self.carphysics.max_speed, self.carphysics.max_speed)

        if abs(target_speed) > self.carphysics.max_speed and self.verbose:
            print("Max speed reached, clipping value")

        return speed

    def set_target_accel(self, target_accel):
        """
        set target accel
        """
        accel = np.clip(target_accel, -self.carphysics.max_decel, self.carphysics.max_accel)

        if self.verbose:
            if target_accel > self.carphysics.max_accel:
                print("Max acceleration reached, clipping value")
            else if target_accel < self.carphysics.max_decel
                print("Max deceleration reached, clipping value")

        return accel

    def get_steering_control(self, steering_angle):
        """
        Basic steering control
        """
        steer_command = self.set_target_steering_angle(steering_angle) / self.carphysics.max_steering_angle
        return steer_command

    def get_throttle_brake_control(self, accel_target):
        """
        get throttle brake output based on acceleration input
        """
        # the driving impedance moves the 'zero' acceleration border
        # Interpretation: To reach a zero acceleration the throttle has to pushed
        # down for a certain amount
        accel_target  = self.set_target_accel(accel_target)
        throttle_lower_border = self.carphysics.get_vehicle_driving_impedance_acceleration(self.vehicle_state, self.info.output.reverse)

        # the engine lay off acceleration defines the size of the coasting area
        # Interpretation: The engine already prforms braking on its own;
        #  therefore pushing the brake is not required for small decelerations
        brake_upper_border = throttle_lower_border + self.carphysics.engine_impedance

        brake, throttle = 0.0, 0.0

        if accel_target > throttle_lower_border:
            # Acceleration mode, car needs to give more throttle than acc_desired based on losses
            # the value has to be normed to max_pedal
            # be aware: is not required to take throttle_lower_border into the scaling factor,
            # because that border is in reality a shift of the coordinate system
            # the global maximum acceleration can practically not be reached anymore because of
            # driving impedance
            throttle = ((accel_target - throttle_lower_border) / abs(self.carphysics.max_accel))
        elif accel_target > brake_upper_border:
            # Coasting mode, the car will itself slow down in this region
            pass
        else:
            # braking mode, we need to apply lesser brakes than required by iLQR cause we already have other losses 
            brake = ((brake_upper_border - accel_target) / abs(self.carphysics.max_decel))

        return throttle, brake