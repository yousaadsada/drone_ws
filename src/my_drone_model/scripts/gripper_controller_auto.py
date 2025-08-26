#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import threading
import tkinter as tk
from std_msgs.msg import Bool

class gripper_command():
    def __init__(self):
        rospy.init_node('gripper_commander')

        self.running = True
        self.command_value = 0.0

        self.sub_gripper_command = rospy.Subscriber('/gripper_command', Bool, self.sub_gripper_command_callback, queue_size=1)
        self.pub_left = rospy.Publisher('/quadrotor/left_finger_controller/command', Float64, queue_size=10)
        self.pub_right = rospy.Publisher('/quadrotor/right_finger_controller/command', Float64, queue_size=10)

        self.rate = rospy.Rate(100)  # 1 Hz

        rospy.loginfo("Publishing left finger command: %f", self.command_value)

        self.publish_wrench_thread = threading.Thread(target=self.publish_gripper_command_thread_callback)
        self.publish_wrench_thread.daemon = True
        self.publish_wrench_thread.start()


    def sub_gripper_command_callback(self, msg):
        if msg.data == True:
            self.command_value = 0.01
        else:
            self.command_value = 0.0


    def publish_gripper_command_thread_callback(self):

        while self.running:
            
            self.pub_left.publish(self.command_value)
            self.pub_right.publish(self.command_value)
            self.rate.sleep()

if __name__ == '__main__':
    node = gripper_command()
    rospy.spin()
