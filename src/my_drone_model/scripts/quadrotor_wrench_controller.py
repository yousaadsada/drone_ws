#!/usr/bin/env python

import rospy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float32MultiArray
import math
import time

class PID:
    def __init__(self, kp, ki, kd, limit=10.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = None

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = None

    def update(self, target, current):
        error = target - current
        current_time = time.time()
        dt = 0.01 if self.last_time is None else (current_time - self.last_time)
        self.last_time = current_time

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(min(output, self.limit), -self.limit)


class QuadrotorController:
    def __init__(self):
        rospy.init_node('quadrotor_pid_controller')

        self.force_pub = rospy.Publisher('/force', WrenchStamped, queue_size=1)
        rospy.Subscriber('/quadrotor_cmd', Float32MultiArray, self.cmd_callback)

        # PID for roll (x torque), pitch (y torque), yaw (z torque), vertical (z force)
        self.pid_roll  = PID(kp=1.0, ki=0.0, kd=0.1)
        self.pid_pitch = PID(kp=1.0, ki=0.0, kd=0.1)
        self.pid_yaw   = PID(kp=1.0, ki=0.0, kd=0.05)
        self.pid_z     = PID(kp=10.0, ki=0.0, kd=1.0)

        # State assumption (simplified for now)
        self.current_pitch = 0.0
        self.current_roll = 0.0
        self.current_z_velocity = 0.0
        self.current_yaw_rate = 0.0

        self.rate = rospy.Rate(100)

    def cmd_callback(self, msg):
        # Input: x, y, z, yaw in range [-100, 100]
        cmd = msg.data
        if len(cmd) != 4:
            rospy.logwarn("Invalid command length")
            return

        x_input, y_input, z_input, yaw_input = cmd

        # Map inputs
        target_pitch = max(min(x_input / 100.0 * 20.0, 20.0), -20.0) * math.pi / 180  # radians
        target_roll  = max(min(y_input / 100.0 * 20.0, 20.0), -20.0) * math.pi / 180
        target_z_vel = max(min(z_input / 100.0 * 1.0, 1.0), -1.0)
        target_yaw_rate = max(min(yaw_input / 100.0 * 1.0, 1.0), -1.0)

        # Apply PID
        torque_x = self.pid_roll.update(target_roll, self.current_roll)
        torque_y = self.pid_pitch.update(target_pitch, self.current_pitch)
        torque_z = self.pid_yaw.update(target_yaw_rate, self.current_yaw_rate)
        force_z = self.pid_z.update(target_z_vel, self.current_z_velocity)

        # Publish
        wrench = WrenchStamped()
        wrench.header.stamp = rospy.Time.now()
        wrench.wrench.force.x = 0.0
        wrench.wrench.force.y = 0.0
        wrench.wrench.force.z = force_z
        wrench.wrench.torque.x = torque_x
        wrench.wrench.torque.y = torque_y
        wrench.wrench.torque.z = torque_z

        self.force_pub.publish(wrench)

    def spin(self):
        while not rospy.is_shutdown():
            self.rate.sleep()


if __name__ == '__main__':
    try:
        controller = QuadrotorController()
        controller.spin()
    except rospy.ROSInterruptException:
        pass
