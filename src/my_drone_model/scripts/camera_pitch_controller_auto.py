#!/usr/bin/env python
import rospy
import tf
from std_msgs.msg import Float64
import math
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates
import threading
import tkinter as tk
from tkinter import messagebox

class CameraPitch:
    def __init__(self):
        rospy.init_node('camera_pitch')

        self.quadrotor_pitch = 0.0
        self.ref_camera_pitch = 0.0
        self.correct_camera_pitch = 0.0

        self.subscribe_ref_camera_pitch = rospy.Subscriber('/ref_camera_pitch', Float64, self.subscribe_ref_camera_pitch_callback)
        self.subscribe_quadrotor_state = rospy.Subscriber('/gazebo/model_states', ModelStates, self.subscribe_model_states_callback)
        self.publish_correct_camera_pitch = rospy.Publisher('/quadrotor/quadrotor_camera_pitch_controller/command', Float64, queue_size=1)

        self.publish_correct_camera_pitch_thread = threading.Thread(target=self.publish_correct_camera_pitch_thread_callback)
        self.publish_correct_camera_pitch_thread.daemon = True
        self.publish_correct_camera_pitch_thread.start()


    def subscribe_model_states_callback(self, msg):
        try:
            index = msg.name.index("quadrotor")
            orientation_q = msg.pose[index].orientation
            quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            _, pitch, _ = euler_from_quaternion(quaternion)
            self.quadrotor_pitch = pitch
        except ValueError:
            rospy.logwarn("quadrotor not found in model_states")

    def subscribe_ref_camera_pitch_callback(self, msg):
        self.ref_camera_pitch = msg.data

    def publish_correct_camera_pitch_thread_callback(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.correct_camera_pitch = self.ref_camera_pitch - self.quadrotor_pitch
            self.publish_correct_camera_pitch.publish(self.correct_camera_pitch)
            rate.sleep()



if __name__ == '__main__':
    node = CameraPitch()
    rospy.spin()
