import sys
import pdb
import numpy as np
import matplotlib.pyplot as plt

import carla
# from simple_pid import PID

from simulator.vehicle_physics import VehiclePhysicsInfo


class LowLevelController():
    def __init__(self, carla_vehicle_info, verbose=False, plot=False):
        self.carla_vehicle_info = carla_vehicle_info
        self.carphysics = VehiclePhysicsInfo(self.carla_vehicle_info)
        self.verbose = verbose
        self.plot = plot
        if plot:
            self.current_states = None
            self.desired_accel = None

    def get_control(self, vehicle_state, accel, steering_angle):
        self.vehicle_state = vehicle_state
        self.reverse = vehicle_state[1,0] < 0

        if self.plot:
            if self.current_states is None:
                self.current_states = np.array([[vehicle_state[1,0], vehicle_state[1,1], vehicle_state[4,0], vehicle_state[4,1]]])
                self.desired_accel = np.array([[accel]])
            else:
                self.current_states = np.vstack((self.current_states, np.array([[vehicle_state[1,0], vehicle_state[1,1], vehicle_state[4,0], vehicle_state[4,1]]])))
                self.desired_accel = np.vstack((self.desired_accel, np.array([[accel]])))

        control = carla.VehicleControl()
        
        control.throttle, control.brake = self.get_throttle_brake_control(accel)
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
            elif target_accel < -self.carphysics.max_decel:
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
        throttle_lower_border = self.carphysics.get_vehicle_driving_impedance_acceleration(self.vehicle_state, self.reverse)

        # the engine lay off acceleration defines the size of the coasting area
        # Interpretation: The engine already prforms braking on its own;
        #  therefore pushing the brake is not required for small decelerations
        # Currently deceleration due to impedence is 0.239183598 m/s^2
        brake_upper_border = throttle_lower_border + self.carphysics.engine_impedance 
        if self.verbose:
            print('Throttle Lower Border: {} \nBrake Upper Border: {}'.format(throttle_lower_border, brake_upper_border))
        brake, throttle = 0.0, 0.0

        if accel_target > throttle_lower_border:
            # Acceleration mode, car needs to give more throttle than acc_desired based on losses
            # the value has to be normed to max_pedal
            # be aware: is not required to take throttle_lower_border into the scaling factor,
            # because that border is in reality a shift of the coordinate system
            # the global maximum acceleration can practically not be reached anymore because of
            # driving impedance
            throttle = ((accel_target - throttle_lower_border) / abs(self.carphysics.max_accel))
            if self.verbose:
                print("Throttle Mode: {}".format(throttle))
        elif accel_target > brake_upper_border:
            # Coasting mode, the car will itself slow down in this region
            pass
        else:
            # braking mode, we need to apply lesser brakes than required by iLQR cause we already have other losses 
            brake = ((brake_upper_border - accel_target) / abs(self.carphysics.max_decel))
            if self.verbose:
                print("Brake Mode: {}".format(brake))

        return throttle, brake

    def plot_pid(self):
        if self.plot:
            plt.figure(0)
            plt.plot(np.arange(len(self.current_states)), self.current_states[:,2], color='g', label='longitudinal_acc')
            plt.plot(np.arange(len(self.current_states)), self.current_states[:,3], color='b', label='lateral_acc')
            plt.plot(np.arange(len(self.current_states)), self.desired_accel, color='r')
            plt.legend()
            plt.figure(1)
            plt.plot(np.arange(len(self.current_states)), self.current_states[:,0], color='g', label='longitudinal_vel')
            plt.plot(np.arange(len(self.current_states)), self.current_states[:,1], color='b', label='lateral_vel')
            plt.show()
