#!/usr/bin/env python3

"""
    A base controller class for the Arduino microcontroller

    Borrowed heavily from Mike Feguson's ArbotiX base_controller.py code.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2010 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses
"""

import os
from math import sin, cos, pi, degrees, atan2, asin

from rclpy.duration import Duration
from geometry_msgs.msg import Quaternion, Twist, Pose, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty, Int32
import tf2_ros

from encoder_msgs.msg import Encoder
from own_msgs.msg import SlamSimulation, ResetOdom


def euler_from_quaternion(q):
    """Convert a quaternion into euler angles (roll, pitch, yaw)."""
    x, y, z, w = q
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = pi / 2.0 if sinp > 0 else -pi / 2.0
    else:
        pitch = asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class BaseController:
    def __init__(self, node, arduino, base_frame, name="base_controllers"):
        self.node = node
        self.arduino = arduino
        self.name = name
        self.base_frame = base_frame

        self.node.declare_parameter('base_controller_rate', 10)
        self.node.declare_parameter('base_controller_timeout', 1.0)
        self.node.declare_parameter('publish_odom', True)
        self.node.declare_parameter('odom_frame', 'odom')
        self.node.declare_parameter('cartographer_slam', False)
        self.node.declare_parameter('use_imu_onboard', False)
        self.node.declare_parameter('wheel_diameter', 0.0)
        self.node.declare_parameter('wheel_track', 0.0)
        self.node.declare_parameter('encoder_resolution', 0)
        self.node.declare_parameter('gear_reduction', 1.0)
        self.node.declare_parameter('Kp', 20)
        self.node.declare_parameter('Kd', 12)
        self.node.declare_parameter('Ki', 0)
        self.node.declare_parameter('Ko', 50)
        self.node.declare_parameter('accel_limit', 0.1)

        self.rate = float(self.node.get_parameter('base_controller_rate').value)
        self.timeout = self.node.get_parameter('base_controller_timeout').value
        self.stopped = False
        self.publish_odom = self.node.get_parameter('publish_odom').value
        self.odom_frame = self.node.get_parameter('odom_frame').value
        self.cartographer_slam = self.node.get_parameter('cartographer_slam').value
        self.use_imu_onboard = self.node.get_parameter('use_imu_onboard').value

        pid_params = dict()
        pid_params['wheel_diameter'] = self.node.get_parameter('wheel_diameter').value
        pid_params['wheel_track'] = self.node.get_parameter('wheel_track').value
        pid_params['encoder_resolution'] = self.node.get_parameter('encoder_resolution').value
        pid_params['gear_reduction'] = self.node.get_parameter('gear_reduction').value
        pid_params['Kp'] = self.node.get_parameter('Kp').value
        pid_params['Kd'] = self.node.get_parameter('Kd').value
        pid_params['Ki'] = self.node.get_parameter('Ki').value
        pid_params['Ko'] = self.node.get_parameter('Ko').value

        self.accel_limit = self.node.get_parameter('accel_limit').value
        self.motors_reversed = self.node.get_parameter('motors_reversed').value

        # Set up PID parameters and check for missing values
        self.setup_pid(pid_params)

        # How many encoder ticks are there per meter?
        self.ticks_per_meter = self.encoder_resolution * self.gear_reduction / (self.wheel_diameter * pi)

        # What is the maximum acceleration we will tolerate when changing wheel speeds?
        self.max_accel = self.accel_limit * self.ticks_per_meter / self.rate

        # Track how often we get a bad encoder count (if any)
        self.bad_encoder_count = 0

        from rclpy.duration import Duration
        now = self.node.get_clock().now()
        self.then = now  # time for determining dx/dy
        self.t_delta = Duration(seconds=1.0 / self.rate)
        self.t_next = now + self.t_delta

        # Internal data
        self.enc_left = None            # encoder readings
        self.enc_right = None
        self.enc0 = None
        self.enc1 = None
        self.enc2 = None
        self.enc3 = None
        self.x = 0.0                    # position in xy plane
        self.y = 0.0
        self.th = 0.0                   # rotation in radians

        self.v0 = 0
        self.v1 = 0
        self.v2 = 0
        self.v3 = 0
        self.v_des0 = 0
        self.v_des1 = 0
        self.v_des2 = 0
        self.v_des3 = 0

        self.v_left = 0
        self.v_right = 0
        self.v_des_left = 0  # cmd_vel setpoint
        self.v_des_right = 0

        self.ENCODER_LOW_WRAP_FACTOR = 0.3
        self.ENCODER_HIGH_WRAP_FACTOR = 0.7
        self.ENCODER_MAX = 65535
        self.ENCODER_MIN = 0
        self.low_wrap = self.ENCODER_LOW_WRAP_FACTOR * (self.ENCODER_MAX - self.ENCODER_MIN) + self.ENCODER_MIN
        self.high_wrap = self.ENCODER_HIGH_WRAP_FACTOR * (self.ENCODER_MAX - self.ENCODER_MIN) + self.ENCODER_MIN
        self.last_wheelcountR = 0
        self.last_wheelcountL = 0
        self.multR = 0
        self.multL = 0

        self.lastPosL = 0.0
        self.lastPosR = 0.0
        self.lastPubPosL = 0.0
        self.lastPubPosR = 0.0
        self.nodeStartFlag = True

        self.q0 = 1.0  # W
        self.q1 = 0.0  # X
        self.q2 = 0.0  # Y
        self.q3 = 0.0  # Z

        self.twoKp = 1.0
        self.twoKi = 0
        self.integralFBx = 0
        self.integralFBy = 0
        self.integralFBz = 0
        self.gyroscope_radian = 0.001064  # (2000 / 32768) * (3.14 / 180)
        self.accelerometer = 16384.0   # 32768 / 2
        self.gyroscope_offsets = [0] * 3
        self.accelerometer_offsets = [0] * 3

        self.offsetCounts = 0
        self.offsetFullCounts = 50

        self.last_cmd_vel = now
        self._last_pic_cnt = 0

        # Subscriptions
        self.node.create_subscription(Twist, 'cmd_vel', self.cmdVelCallback, 5)
        self.node.create_subscription(SlamSimulation, '/slam_simulation', self.simulationCallback, 5)
        self.node.create_subscription(ResetOdom, '/reset_odom', self.resetodomCallback, 5)

        # Clear any old odometry info
        self.arduino.reset_encoders()

        # Set up the odometry broadcaster
        self.odomPub = self.node.create_publisher(Odometry, 'odom', 5)
        self.encoderPub = self.node.create_publisher(Encoder, 'encoder', 10)
        self.imuPub = None
        if self.use_imu_onboard:
            self.imuPub = self.node.create_publisher(Imu, '/imu', 10)
        self.odomBroadcaster = tf2_ros.TransformBroadcaster(self.node)
        self.take_picture_evt_pub = self.node.create_publisher(Empty, "take_picture/event", 10)

        self.node.get_logger().info("Started base controller for a base of " + str(self.wheel_track) + "m wide with " + str(self.encoder_resolution) + " ticks per rev")
        self.node.get_logger().info("Publishing odometry data at: " + str(self.rate) + " Hz using " + str(self.base_frame) + " as base frame")

    def setup_pid(self, pid_params):
        # Check to see if any PID parameters are missing
        missing_params = False
        for param in ['wheel_diameter', 'wheel_track', 'encoder_resolution']:
            value = pid_params[param]
            if value == 0 or value == 0.0 or value == "":
                print("*** PID Parameter " + param + " is missing. ***")
                missing_params = True

        if missing_params:
            os._exit(1)

        self.wheel_diameter = pid_params['wheel_diameter']
        self.wheel_track = pid_params['wheel_track']
        self.encoder_resolution = pid_params['encoder_resolution']
        self.gear_reduction = pid_params['gear_reduction']

        self.Kp = pid_params['Kp']
        self.Kd = pid_params['Kd']
        self.Ki = pid_params['Ki']
        self.Ko = pid_params['Ko']

        self.arduino.update_pid(self.Kp, self.Kd, self.Ki, self.Ko)

    def imu(self, dt):
        msg = Imu()
        msg.header.frame_id = "onboard_imu_link"
        msg.header.stamp = self.node.get_clock().now().to_msg()
        try:
            ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw = self.arduino.get_imu()
        except Exception as err:
            print(err)
            self.node.get_logger().error("IMU exception count: " + str(self.bad_encoder_count))
            return
        ax_ned = ax_raw / self.accelerometer
        ay_ned = ay_raw / self.accelerometer
        az_ned = az_raw / self.accelerometer
        gx_ned = gx_raw * self.gyroscope_radian
        gy_ned = gy_raw * self.gyroscope_radian
        gz_ned = gz_raw * self.gyroscope_radian

        ax = -ay_ned
        ay = ax_ned
        az = az_ned
        gx = -gy_ned
        gy = gx_ned
        gz = gz_ned

        if self.offsetCounts < self.offsetFullCounts:
            self.offsetCounts = self.offsetCounts + 1
            self.gyroscope_offsets[0] += gx
            self.gyroscope_offsets[1] += gy
            self.gyroscope_offsets[2] += gz
            self.accelerometer_offsets[0] += ax
            self.accelerometer_offsets[1] += ay
            if self.offsetCounts == self.offsetFullCounts:
                for i in range(3):
                    self.gyroscope_offsets[i] = self.gyroscope_offsets[i] / self.offsetFullCounts
                for j in range(2):
                    self.accelerometer_offsets[j] = self.accelerometer_offsets[j] / self.offsetFullCounts
        else:
            gx = gx - self.gyroscope_offsets[0]
            gy = gy - self.gyroscope_offsets[1]
            gz = gz - self.gyroscope_offsets[2]

            if not ((ax == 0.0) and (ay == 0.0) and (az == 0.0)):
                # Normalise accelerometer measurement
                recipNorm = (ax * ax + ay * ay + az * az) ** -.5
                ax *= recipNorm
                ay *= recipNorm
                az *= recipNorm
                # Estimated direction of gravity and vector perpendicular to magnetic flux
                halfvx = self.q1 * self.q3 - self.q0 * self.q2
                halfvy = self.q0 * self.q1 + self.q2 * self.q3
                halfvz = self.q0 * self.q0 - 0.5 + self.q3 * self.q3
                # Error is sum of cross product between estimated and measured direction of gravity
                halfex = (ay * halfvz - az * halfvy)
                halfey = (az * halfvx - ax * halfvz)
                halfez = (ax * halfvy - ay * halfvx)
                # Compute and apply integral feedback (if enabled)
                if self.twoKi > 0:
                    self.integralFBx += self.twoKi * halfex * dt
                    self.integralFBy += self.twoKi * halfey * dt
                    self.integralFBz += self.twoKi * halfez * dt
                    gx += self.integralFBx
                    gy += self.integralFBy
                    gz += self.integralFBz
                else:
                    self.integralFBx = 0
                    self.integralFBy = 0
                    self.integralFBz = 0

                # Apply proportional feedback
                gx += self.twoKp * halfex
                gy += self.twoKp * halfey
                gz += self.twoKp * halfez

            # Integrate rate of change of quaternion
            gx *= (0.5 * dt)
            gy *= (0.5 * dt)
            gz *= (0.5 * dt)
            qa = self.q0
            qb = self.q1
            qc = self.q2
            self.q0 += (-qb * gx - qc * gy - self.q3 * gz)
            self.q1 += (qa * gx + qc * gz - self.q3 * gy)
            self.q2 += (qa * gy - qb * gz + self.q3 * gx)
            self.q3 += (qa * gz + qb * gy - qc * gx)
            # Normalise quaternion
            recipNorm = (self.q0 * self.q0 + self.q1 * self.q1 + self.q2 * self.q2 + self.q3 * self.q3) ** -.5
            self.q0 *= recipNorm
            self.q1 *= recipNorm
            self.q2 *= recipNorm
            self.q3 *= recipNorm

            # Fill message
            msg.orientation.x = self.q1
            msg.orientation.y = self.q2
            msg.orientation.z = self.q3
            msg.orientation.w = self.q0
            msg.orientation_covariance[0] = self.q1 * self.q1
            msg.orientation_covariance[0] = self.q2 * self.q2
            msg.orientation_covariance[0] = self.q3 * self.q3

            msg.angular_velocity.x = gx
            msg.angular_velocity.y = gy
            msg.angular_velocity.z = gz

            msg.linear_acceleration.x = (ax - self.accelerometer_offsets[0]) * 9.8
            msg.linear_acceleration.y = (ay - self.accelerometer_offsets[1]) * 9.8
            msg.linear_acceleration.z = az * 9.8

            (r, p, y) = euler_from_quaternion(
                [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])

            roll = r / 3.14159 * 180
            pitch = p / 3.14159 * 180
            yaw = y / 3.14159 * 180

            if self.imuPub is not None:
                self.imuPub.publish(msg)

    def poll(self):
        now = self.node.get_clock().now()
        if now > self.t_next:
            # Read the encoders
            try:
                left_enc, right_enc, pic_cnt = self.arduino.get_encoder_counts()
            except Exception as e:
                print(e)
                self.bad_encoder_count += 1
                self.node.get_logger().error("Encoder exception count: " + str(self.bad_encoder_count))
                return

            if pic_cnt != self._last_pic_cnt:
                self._last_pic_cnt = pic_cnt
                self.take_picture_evt_pub.publish(Empty())

            if right_enc < self.low_wrap and self.last_wheelcountR > self.high_wrap:
                self.multR = self.multR + 1
            elif right_enc > self.high_wrap and self.last_wheelcountR < self.low_wrap:
                self.multR = self.multR - 1
            posR = right_enc + self.multR * (self.ENCODER_MAX - self.ENCODER_MIN)
            self.last_wheelcountR = right_enc

            if left_enc < self.low_wrap and self.last_wheelcountL > self.high_wrap:
                self.multL = self.multL + 1
            elif left_enc > self.high_wrap and self.last_wheelcountL < self.low_wrap:
                self.multL = self.multL - 1

            posL = left_enc + self.multL * (self.ENCODER_MAX - self.ENCODER_MIN)
            self.last_wheelcountL = left_enc

            posLDiff = 0
            posRDiff = 0

            if self.nodeStartFlag:
                self.nodeStartFlag = False
            else:
                posLDiff = posL - self.lastPosL
                posRDiff = posR - self.lastPosR

            self.lastPubPosL += posLDiff
            self.lastPubPosR += posRDiff
            self.lastPosL = posL
            self.lastPosR = posR

            dt = now - self.then
            self.then = now
            dt = dt.nanoseconds / 1e9

            dright = posRDiff / self.ticks_per_meter
            dleft = posLDiff / self.ticks_per_meter

            dxy_ave = (dright + dleft) / 2.0
            dth = (dright - dleft) / self.wheel_track
            vxy = dxy_ave / dt
            vth = dth / dt

            if (dxy_ave != 0):
                dx = cos(dth) * dxy_ave
                dy = -sin(dth) * dxy_ave
                self.x += (cos(self.th) * dx - sin(self.th) * dy)
                self.y += (sin(self.th) * dx + cos(self.th) * dy)

            if (dth != 0):
                self.th += dth

            if self.use_imu_onboard:
                self.imu(dt)

            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin(self.th / 2.0)
            quaternion.w = cos(self.th / 2.0)

            if self.publish_odom:
                # Create the odometry transform frame broadcaster.
                t = TransformStamped()
                t.header.stamp = self.node.get_clock().now().to_msg()
                t.header.frame_id = self.odom_frame
                t.child_frame_id = self.base_frame
                t.transform.translation.x = self.x
                t.transform.translation.y = self.y
                t.transform.translation.z = 0.0
                t.transform.rotation = quaternion
                self.odomBroadcaster.sendTransform(t)

                odom = Odometry()
                odom.header.frame_id = self.odom_frame
                odom.child_frame_id = self.base_frame
                odom.header.stamp = now.to_msg()
                odom.pose.pose.position.x = self.x
                odom.pose.pose.position.y = self.y
                odom.pose.pose.position.z = 0.0
                odom.pose.pose.orientation = quaternion
                odom.twist.twist.linear.x = vxy
                odom.twist.twist.linear.y = 0.0
                odom.twist.twist.angular.z = vth

                self.odomPub.publish(odom)

            if now > (self.last_cmd_vel + Duration(seconds=self.timeout)):
                self.v_des_left = 0
                self.v_des_right = 0

            if self.v_left < self.v_des_left:
                self.v_left += self.max_accel
                if self.v_left > self.v_des_left:
                    self.v_left = self.v_des_left
            else:
                self.v_left -= self.max_accel
                if self.v_left < self.v_des_left:
                    self.v_left = self.v_des_left

            if self.v_right < self.v_des_right:
                self.v_right += self.max_accel
                if self.v_right > self.v_des_right:
                    self.v_right = self.v_des_right
            else:
                self.v_right -= self.max_accel
                if self.v_right < self.v_des_right:
                    self.v_right = self.v_des_right

            # Set motor speeds in encoder ticks per PID loop
            if not self.stopped:
                self.arduino.drive(int(self.v_left), int(self.v_right), 0)

            self.t_next = now + self.t_delta

    def stop(self):
        self.stopped = True
        self.arduino.stop()
        print("stopping the robot")

    def cmdVelCallback(self, req):
        if not self.publish_odom:
            return
        # Handle velocity-based movement requests
        self.last_cmd_vel = self.node.get_clock().now()

        x = req.linear.x  # m/s
        th = req.angular.z  # rad/s

        if x == 0:
            # Turn in place
            right = th * self.wheel_track * self.gear_reduction / 2.0
            left = -right
        elif th == 0:
            # Pure forward/backward motion
            left = right = x
        else:
            # Rotation about a point in space
            left = x - th * self.wheel_track * self.gear_reduction / 2.0
            right = x + th * self.wheel_track * self.gear_reduction / 2.0

        self.v_des_left = left * 1000
        self.v_des_right = right * 1000

    def simulationCallback(self, msg):
        self.publish_odom = not msg.slam_simulation

    def resetodomCallback(self, msg):
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
