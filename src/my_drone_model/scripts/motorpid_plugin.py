#!/usr/bin/env python

import rospy
import numpy as np
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Float64MultiArray, Float64

class PIDController:
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, setpoint, actual):
        error = setpoint - actual
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

class QuadrotorMotorController:
    def __init__(self):
        rospy.init_node('quadrotor_motor_controller', anonymous=True)
        self.rate = rospy.Rate(100)
        self.dt = 1.0 / 100.0

        self.model_name = rospy.get_param('~model_name', 'quadrotor')
        self.vel_cmd_topic = f"/{self.model_name}/vel_cmd"

        self.kp = rospy.get_param("/rotor1_controller/pid/p", 0.1)
        self.ki = rospy.get_param("/rotor1_controller/pid/i", 0.0)
        self.kd = rospy.get_param("/rotor1_controller/pid/d", 0.0)
        self.pid_controllers = [
            PIDController(kp=self.kp, ki=self.ki, kd=self.kd, dt=self.dt) for _ in range(4)
        ]

        self.desired_speeds = [0.0] * 4
        self.actual_speeds = [0.0] * 4

        self.motor_pubs = [
            rospy.Publisher(f'/{self.model_name}/rotor{i+1}_controller/command', Float64, queue_size=10)
            for i in range(4)
        ]
        self.vel_cmd_pub = rospy.Publisher(self.vel_cmd_topic, Float64MultiArray, queue_size=10)

        rospy.Subscriber(self.vel_cmd_topic, Float64MultiArray, self.vel_cmd_callback)
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.link_states_callback)

        rospy.loginfo(f"[{self.model_name}] Motor controller initialized. Waiting for velocity commands on {self.vel_cmd_topic}...")

    def vel_cmd_callback(self, msg):
        for i in range(min(len(msg.data), 4)):
            self.desired_speeds[i] = msg.data[i]
        rospy.loginfo(f"[{self.model_name}] Received desired speeds: {self.desired_speeds}")

    def link_states_callback(self, msg):
        for i in range(4):
            try:
                idx = msg.name.index(f'{self.model_name}::propeller{i+1}')
                self.actual_speeds[i] = msg.twist[idx].angular.z
            except ValueError:
                pass

    def control_loop(self):
        while not rospy.is_shutdown():
            for i in range(4):
                control_effort = self.pid_controllers[i].compute(self.desired_speeds[i], self.actual_speeds[i])
                self.motor_pubs[i].publish(Float64(control_effort))
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = QuadrotorMotorController()
        controller.control_loop()
    except rospy.ROSInterruptException:
        pass
