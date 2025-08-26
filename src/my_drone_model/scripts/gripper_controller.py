#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import threading
import tkinter as tk

class gripper_command():
    def __init__(self):
        rospy.init_node('gripper_commander')

        self.running = True

        self.pub_left = rospy.Publisher('/quadrotor/left_finger_controller/command', Float64, queue_size=10)
        self.pub_right = rospy.Publisher('/quadrotor/right_finger_controller/command', Float64, queue_size=10)

        self.rate = rospy.Rate(100)  # 1 Hz
        # command_value = -0.01  # Desired position for left finger
        self.command_value = 0.0  # Desired position for left finger

        rospy.loginfo("Publishing left finger command: %f", self.command_value)

        self.publish_wrench_thread = threading.Thread(target=self.publish_gripper_command_thread_callback)
        self.publish_wrench_thread.daemon = True
        self.publish_wrench_thread.start()

        self.gui_thread = threading.Thread(target=self.create_gui)
        self.gui_thread.daemon = True
        self.gui_thread.start()

    def create_gui(self):
  
        self.root = tk.Tk()
        self.root.title("Gripper Control")
        self.root.geometry("200x100")
        
        take_off_button = tk.Button(self.root, text="Drop off", command=self.enable_drop_off)
        take_off_button.pack(pady=20)
        
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.mainloop()

    def enable_drop_off(self):
        """Set flag to enable wrench publishing."""
        self.command_value = 0.01
        rospy.loginfo("Take Off button clicked, enabling wrench publishing.")

    def on_closing(self):
        """Handle window closing."""
        self.running = False
        self.root.destroy()

    
    def publish_gripper_command_thread_callback(self):

        while self.running:
            
            self.pub_left.publish(self.command_value)
            self.pub_right.publish(self.command_value)
            self.rate.sleep()

if __name__ == '__main__':
    node = gripper_command()
    rospy.spin()
