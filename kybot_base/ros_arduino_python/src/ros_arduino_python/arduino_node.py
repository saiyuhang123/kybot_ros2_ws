#!/usr/bin/env python3

"""
    A ROS 2 Node for the Arduino microcontroller

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html
"""

import json
import os
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist

from ros_arduino_python.arduino_driver import Arduino
from ros_arduino_python.arduino_sensors import (
    Ping, GP2D12, DigitalSensor, AnalogSensor,
    PololuMotorCurrent, PhidgetsVoltage, PhidgetsCurrent
)
from ros_arduino_python.base_controller import BaseController
from ros_arduino_msgs.msg import SensorState
from ros_arduino_msgs.srv import (
    ServoWrite, ServoRead, DigitalSetDirection,
    DigitalWrite, DigitalRead, AnalogWrite, AnalogRead
)
from serial.serialutil import SerialException


class ArduinoROS(Node):
    def __init__(self):
        super().__init__('arduino')

        # Declare parameters with defaults
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 57600)
        self.declare_parameter('timeout', 0.5)
        self.declare_parameter('base_frame', 'car_base_link')
        self.declare_parameter('motors_reversed', False)
        self.declare_parameter('rate', 50)
        self.declare_parameter('sensorstate_rate', 10)
        self.declare_parameter('use_base_controller', False)
        self.declare_parameter('sensors', '{}')

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.timeout = self.get_parameter('timeout').value
        self.base_frame = self.get_parameter('base_frame').value
        self.motors_reversed = self.get_parameter('motors_reversed').value
        self.rate = int(self.get_parameter('rate').value)
        self.sensorstate_rate = int(self.get_parameter('sensorstate_rate').value)
        self.use_base_controller = self.get_parameter('use_base_controller').value

        # Set up the time for publishing the next SensorState message
        now = self.get_clock().now()
        self.t_delta_sensors = Duration(seconds=1.0 / self.sensorstate_rate)
        self.t_next_sensors = now + self.t_delta_sensors

        # Initialize a Twist message
        self.cmd_vel = Twist()

        # A cmd_vel publisher so we can stop the robot when shutting down
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 5)

        # The SensorState publisher periodically publishes the values of all sensors
        self.sensorStatePub = self.create_publisher(SensorState, 'sensor_state', 5)

        # Services
        self.create_service(ServoWrite, 'servo_write', self.ServoWriteHandler)
        self.create_service(ServoRead, 'servo_read', self.ServoReadHandler)
        self.create_service(DigitalSetDirection, 'digital_set_direction', self.DigitalSetDirectionHandler)
        self.create_service(DigitalWrite, 'digital_write', self.DigitalWriteHandler)
        self.create_service(DigitalRead, 'digital_read', self.DigitalReadHandler)
        self.create_service(AnalogWrite, 'analog_write', self.AnalogWriteHandler)
        self.create_service(AnalogRead, 'analog_read', self.AnalogReadHandler)

        # Initialize the controller
        self.controller = Arduino(self.port, self.baud, self.timeout, self.motors_reversed)

        # Make the connection
        self.controller.connect()

        self.get_logger().info("Connected to Arduino on port " + self.port + " at " + str(self.baud) + " baud")

        # Reserve a thread lock
        self.mutex = threading.Lock()

        # Initialize any sensors
        self.mySensors = list()

        sensor_params = json.loads(self.get_parameter('sensors').value)
        if sensor_params is None:
            sensor_params = {}

        for name, params in sensor_params.items():
            # Set the direction to input if not specified
            if 'direction' not in params:
                params['direction'] = 'input'

            sensor = None
            if params['type'] == "Ping":
                sensor = Ping(self.controller, name, params['pin'], params['rate'], self.base_frame, node=self)
            elif params['type'] == "GP2D12":
                sensor = GP2D12(self.controller, name, params['pin'], params['rate'], self.base_frame, node=self)
            elif params['type'] == 'Digital':
                sensor = DigitalSensor(self.controller, name, params['pin'], params['rate'], self.base_frame,
                                       direction=params['direction'], node=self)
            elif params['type'] == 'Analog':
                sensor = AnalogSensor(self.controller, name, params['pin'], params['rate'], self.base_frame,
                                      direction=params['direction'], node=self)
            elif params['type'] == 'PololuMotorCurrent':
                sensor = PololuMotorCurrent(self.controller, name, params['pin'], params['rate'], self.base_frame, node=self)
            elif params['type'] == 'PhidgetsVoltage':
                sensor = PhidgetsVoltage(self.controller, name, params['pin'], params['rate'], self.base_frame, node=self)
            elif params['type'] == 'PhidgetsCurrent':
                sensor = PhidgetsCurrent(self.controller, name, params['pin'], params['rate'], self.base_frame, node=self)

            try:
                if sensor is not None:
                    self.mySensors.append(sensor)
                    self.get_logger().info(name + " " + str(params) + " published on topic " + self.get_name() + "/sensor/" + name)
            except Exception:
                self.get_logger().error("Sensor type " + str(params['type']) + " not recognized.")

        # Initialize the base controller if used
        self.myBaseController = None
        if self.use_base_controller:
            self.myBaseController = BaseController(self, self.controller, self.base_frame, self.get_name() + "_base_controller")

    def run(self):
        # Start polling the sensors and base controller
        while rclpy.ok():
            for sensor in self.mySensors:
                with self.mutex:
                    sensor.poll(self)

            if self.myBaseController is not None:
                with self.mutex:
                    self.myBaseController.poll()

            # Publish all sensor values on a single topic for convenience
            now = self.get_clock().now()

            if now > self.t_next_sensors:
                msg = SensorState()
                msg.header.frame_id = self.base_frame
                msg.header.stamp = now.to_msg()
                for i in range(len(self.mySensors)):
                    msg.name.append(self.mySensors[i].name)
                    msg.value.append(self.mySensors[i].value)
                try:
                    self.sensorStatePub.publish(msg)
                except Exception:
                    pass

                self.t_next_sensors = now + self.t_delta_sensors

            time.sleep(1.0 / self.rate)

    # Service callback functions
    def ServoWriteHandler(self, request, response):
        self.controller.servo_write(request.id, request.value)
        return response

    def ServoReadHandler(self, request, response):
        pos = self.controller.servo_read(request.id)
        response.value = pos
        return response

    def DigitalSetDirectionHandler(self, request, response):
        self.controller.pin_mode(request.pin, request.direction)
        return response

    def DigitalWriteHandler(self, request, response):
        self.controller.digital_write(request.pin, request.value)
        return response

    def DigitalReadHandler(self, request, response):
        value = self.controller.digital_read(request.pin)
        response.value = value
        return response

    def AnalogWriteHandler(self, request, response):
        self.controller.analog_write(request.pin, request.value)
        return response

    def AnalogReadHandler(self, request, response):
        value = self.controller.analog_read(request.pin)
        response.value = value
        return response

    def shutdown(self):
        self.get_logger().info("Shutting down Arduino Node...")

        # Stop the robot
        try:
            if self.myBaseController is not None:
                self.myBaseController.stop()
            time.sleep(2)
        except Exception as e:
            print(e)
            pass

        # Close the serial port
        try:
            self.controller.close()
        except Exception:
            print("close error")
        finally:
            self.get_logger().info("Serial port closed.")
            os._exit(0)


def main(args=None):
    rclpy.init(args=args)
    try:
        myArduino = ArduinoROS()
    except SerialException:
        rclpy.logging.get_logger('arduino').error("Serial exception trying to open port.")
        os._exit(0)

    # 节点必须 spin,cmd_vel 订阅和服务才能响应;Rate.sleep 也依赖 executor,否则睡死
    spin_thread = threading.Thread(target=rclpy.spin, args=(myArduino,), daemon=True)
    spin_thread.start()

    try:
        myArduino.run()
    except KeyboardInterrupt:
        pass
    finally:
        myArduino.shutdown()
        myArduino.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
