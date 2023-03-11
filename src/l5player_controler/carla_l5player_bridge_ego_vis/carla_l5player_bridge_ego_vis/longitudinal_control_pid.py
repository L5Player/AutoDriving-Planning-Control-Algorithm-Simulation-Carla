#!/usr/bin/env python
#
# Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.


from __future__ import print_function

# import datetime
import math
import rclpy

from carla_msgs.msg import CarlaEgoVehicleControl
from nav_msgs.msg import Odometry
import numpy as np
import threading
from rclpy.node import Node
# ==============================================================================
# -- L5playerControl -----------------------------------------------------------
# ==============================================================================

# class L5playerControl(CompatibleNode):
class L5playerControl(Node):
    """
    Handle input events
    """
    def __init__(self):
        super().__init__("L5playerControl")
        self.role_name = "ego_vehicle"
        self.data_lock = threading.Lock()
        self._control = CarlaEgoVehicleControl()
        self._steer_cache = 0.0

        fast_qos = 10

        self.vehicle_control_manual_override = True

        self.vehicle_control_publisher = self.create_publisher(
            CarlaEgoVehicleControl,
            "/carla/{}/vehicle_control_cmd".format(self.role_name),
            qos_profile=fast_qos)
        
        self._odometry_subscriber = self.create_subscription(
            Odometry,
            "/carla/{}/odometry".format(self.role_name),
            self.odometry_cb,
            qos_profile=10)
        
        self.timer_pid_iteration = self.create_timer(0.08, self._on_new_carla_frame)
        
        self._K_P = 0.206
        self._K_D = 0.515
        self._K_I = 0.0206
        self.error = 0.0
        self.error_integral = 0.0
        self.error_derivative = 0.0
        
        self._current_speed = 0
        self._current_pose = None
        
        self.target_speed = 30 # km/h

    def odometry_cb(self, odometry_msg):
        with self.data_lock:
            self.get_logger().info("I hear imu.")
            self._current_pose = odometry_msg.pose.pose
            self._current_speed = math.sqrt(odometry_msg.twist.twist.linear.x ** 2 +
                                            odometry_msg.twist.twist.linear.y ** 2 +
                                            odometry_msg.twist.twist.linear.z ** 2) * 3.6

    def _on_new_carla_frame(self):
        """
        callback on new frame

        As CARLA only processes one vehicle control command per tick,
        send the current from within here (once per frame)
        """
        acceleration_cmd = self._pid_run_step(self.target_speed, self._current_speed)
        self.get_logger().info("acceleration_cmd: {}".format(acceleration_cmd))
        try:
            self._parse_vehicle_keys(acceleration_cmd, 0.0, 0.0)
            self.vehicle_control_publisher.publish(self._control)
        except Exception as error:
            self.node.logwarn("Could not send vehicle control: {}".format(error))

    def _pid_run_step(self, target_speed, _current_speed):
        """
        Estimate the throttle of the vehicle based on the PID equations

        :param target_speed:  target speed in Km/h
        :param current_speed: current speed of the vehicle in Km/h
        :return: throttle control in the range [0, 1]
        """
        previous_error = self.error
        self.error = target_speed - _current_speed
        self.get_logger().info("velocity error: {}".format(self.error))
        # restrict integral term to avoid integral windup
        self.error_integral = np.clip(self.error_integral + self.error, -40.0, 40.0)
        self.error_derivative = self.error - previous_error
        output = self._K_P * self.error + self._K_I * self.error_integral + self._K_D * self.error_derivative
        return np.clip(output, 0.0, 1.0)
    
    def _parse_vehicle_keys(self, throttle, steer, brake):
        """
        parse key events
        """
        self._control.header.stamp = self.get_clock().now().to_msg()
        self._control.throttle = throttle
        self._control.steer = steer  # round(self._steer_cache, 1)
        self._control.brake = brake
        self._control.hand_brake = False
        self._control.reverse = False
        self._control.gear = 1
        self._control.manual_gear_shift = False
# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================

def main(args=None):
    rclpy.init(args=args)

    l5player_control_node = L5playerControl()
    
    rclpy.spin(l5player_control_node)
    
    l5player_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

