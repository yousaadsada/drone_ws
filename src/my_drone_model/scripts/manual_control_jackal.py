

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
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.linear_speed = 0.5
        self.angular_speed = 1.0

        # Start GUI in a separate thread
        self.gui_thread = threading.Thread(target=self.run_gui)
        self.gui_thread.daemon = True
        self.gui_thread.start()

        self.publish_jackal_vel_thread= threading.Thread(target=self.publish_jackal_vel_thread_callback)
        self.publish_jackal_vel_thread.daemon = True
        self.publish_jackal_vel_thread.start()

    def run_gui(self):
        self.root = tk.Tk()
        self.root.title("Jackal WASD Control")
        self.root.geometry("300x300")

        # W button
        w_button = tk.Button(self.root, text="W", width=10, height=2)
        w_button.grid(row=0, column=1)
        w_button.bind('<ButtonPress-1>', lambda event: self.set_velocity(linear=self.linear_speed))
        w_button.bind('<ButtonRelease-1>', lambda event: self.stop())

        # A button
        a_button = tk.Button(self.root, text="A", width=10, height=2)
        a_button.grid(row=1, column=0)
        a_button.bind('<ButtonPress-1>', lambda event: self.set_velocity(angular=self.angular_speed))
        a_button.bind('<ButtonRelease-1>', lambda event: self.stop())

        # S button
        s_button = tk.Button(self.root, text="S", width=10, height=2)
        s_button.grid(row=1, column=1)
        s_button.bind('<ButtonPress-1>', lambda event: self.set_velocity(linear=-self.linear_speed))
        s_button.bind('<ButtonRelease-1>', lambda event: self.stop())

        # D button
        d_button = tk.Button(self.root, text="D", width=10, height=2)
        d_button.grid(row=1, column=2)
        d_button.bind('<ButtonPress-1>', lambda event: self.set_velocity(angular=-self.angular_speed))
        d_button.bind('<ButtonRelease-1>', lambda event: self.stop())

        self.root.mainloop()

    def set_velocity(self, linear=0.0, angular=0.0):
        self.twist.linear.x = linear
        self.twist.angular.z = angular

    def stop(self, event=None):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        

    def publish_jackal_vel_thread_callback(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.pub.publish(self.twist)
            rate.sleep()



if __name__ == '__main__':
    node = JackalControl()
    rospy.spin()
