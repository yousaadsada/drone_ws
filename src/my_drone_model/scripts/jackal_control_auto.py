

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
from geometry_msgs.msg import Twist

class JackalControl:
    def __init__(self):
        rospy.init_node('jackal_control')

        self.twist = Twist()
        self.sub = rospy.Subscriber('/jackal_command', Twist, self.sub_callback, queue_size=1)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.publish_jackal_vel_thread= threading.Thread(target=self.publish_jackal_vel_thread_callback)
        self.publish_jackal_vel_thread.daemon = True
        self.publish_jackal_vel_thread.start()

    def sub_callback(self, msg):
        self.twist.linear.x = msg.linear.x
        self.twist.angular.z = msg.angular.z


    def publish_jackal_vel_thread_callback(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.pub.publish(self.twist)
            rate.sleep()



if __name__ == '__main__':
    node = JackalControl()
    rospy.spin()
