import rospy
import time
import os
import threading
import csv
from pynput.keyboard import Listener, Key
import logging
import math
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from my_custom_msgs.msg import PCMD
from my_custom_msgs.msg import DroneState
from tf.transformations import euler_from_quaternion
import tkinter as tk
from tkinter import *
from pynput import keyboard
from gazebo_msgs.msg import ModelStates, LinkStates


class CollectData:
    def __init__(self):
        rospy.init_node('collect_data')
        self.running = True
        self.write_data_flag = False
        self.freq = 50
        self.test_axis = None        
        self.csv_file = None  # Initialize csv_file as None

        self.flag = 'manual_control'

        self.cumulative_time_stamp = 0.0
        self.time_stamp = 0.0
        self.collect_drone_state = DroneState()
        self.pcmd_test = PCMD()
        self.pcmd_pub = PCMD()

        self.pitch = 0
        self.roll = 0
        self.yaw = 0
        self.gaz = 0
        self.takeoff = False
        self.land = False

        # MANUAL MODE
        self.x_manual = 0
        self.y_manual = 0
        self.z_manual = 0
        self.yaw_manual = 0


        self.pcmd_publisher = rospy.Publisher('/pcmd', PCMD, queue_size=1)
        rospy.Subscriber('/gazebo/link_states_throttled', LinkStates, self.subcribe_quadrotor_msg_callback)


        self.data_dir = os.path.join(os.path.expanduser("~"), 'drone_ws', 'data', 'drone_state_function')
        os.makedirs(self.data_dir, exist_ok=True) 

        publish_pcmd_thread = threading.Thread(target=self.publish_pcmd_thread_callback)
        publish_pcmd_thread.daemon = True
        publish_pcmd_thread.start()
        
        save_data_thread = threading.Thread(target=self.save_data_thread_callback)
        save_data_thread.daemon = True
        save_data_thread.start()

        gui_thread = threading.Thread(target=self.run_gui)
        gui_thread.daemon = True
        gui_thread.start()

        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.listener.start()

    def run_gui(self):
        self.root = tk.Tk()
        self.root.title("Select Axis")

        tk.Label(self.root, text="Choose Axis").pack()

        tk.Button(self.root, text="X", command=lambda: self.set_axis('x')).pack()
        tk.Button(self.root, text="Y", command=lambda: self.set_axis('y')).pack()
        tk.Button(self.root, text="Z", command=lambda: self.set_axis('z')).pack()
        tk.Button(self.root, text="Yaw", command=lambda: self.set_axis('yaw')).pack()

        self.label = tk.Label(self.root, text="Current axis: None")
        self.label.pack()

        self.root.mainloop()

    def set_axis(self, axis):
        self.test_axis = axis
        rospy.loginfo(f"Axis selected: {self.test_axis}")
        self.label.config(text=f"Current axis: {self.test_axis}")

        self.is_test_on = True
        self.flag = 'collect_data'
        self.csv_file = os.path.join(self.data_dir, f'state_data_{self.test_axis}.csv')
        self.write_csv_header()  # Write CSV header after setting the file name
        self.process()

    def write_csv_header(self):
        with open(self.csv_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['elapsed_time',
                             'roll', 'roll_speed', 'pitch', 'pitch_speed',
                             'z', 'z_speed', 'yaw', 'yaw_speed',
                             'control_x', 'control_y', 'control_z', 'control_yaw'])

    def process(self):

        if self.running:
            try:
                time.sleep(2)  # Additional delay if needed
                self.write_data_flag = True 

                for _ in range(1):

                    if self.test_axis == 'x':
                        self.collect_and_publish_pcmd(duration=1.0, x_input=10, y_input=0, z_input=0, yaw_input=0)
                        self.collect_and_publish_pcmd(duration=1.0, x_input=0, y_input=0, z_input=0, yaw_input=0)

                    elif self.test_axis == 'y':
                        self.collect_and_publish_pcmd(duration=1.0, x_input=0, y_input=10, z_input=0, yaw_input=0)
                        self.collect_and_publish_pcmd(duration=1.0, x_input=0, y_input=0, z_input=0, yaw_input=0)

                    elif self.test_axis == 'z':
                        self.collect_and_publish_pcmd(duration=1.0, x_input=0, y_input=0, z_input=10, yaw_input=0)
                        self.collect_and_publish_pcmd(duration=1.0, x_input=0, y_input=0, z_input=0, yaw_input=0)

                    elif self.test_axis == 'yaw':
                        self.collect_and_publish_pcmd(duration=1.0, x_input=0, y_input=0, z_input=0, yaw_input=10)
                        self.collect_and_publish_pcmd(duration=1.0, x_input=0, y_input=0, z_input=0, yaw_input=0)

                self.write_data_flag = False  # Stop writing data after test
                self.is_control_on = False
                self.is_manual_on = True

                self.x_control = 0
                self.y_control = 0
                self.z_control = 0
                self.yaw_control = 0
                
            finally:
                print('Test finished')

    def collect_and_publish_pcmd(self, duration, x_input, y_input, z_input, yaw_input):

        self.pcmd_test.x = x_input
        self.pcmd_test.y = y_input
        self.pcmd_test.z = z_input
        self.pcmd_test.yaw = yaw_input

        time.sleep(duration)
        



    def on_press(self, key):
        try:
            if key.char.lower() in ['w', 's', 'a', 'd', 'r', 'f', 'c', 'x']:
                self.flag = 'manual_control'

            if key.char.lower() == 'w':
                self.x_manual = -10.0 
            elif key.char.lower() == 's':
                self.x_manual = 10.0
            elif key.char.lower() == 'a':
                self.y_manual = -10.0
            elif key.char.lower() == 'd':
                self.y_manual = 10.0 
            elif key.char.lower() == 'r':
                self.z_manual = 10.0
            elif key.char.lower() == 'f':
                self.z_manual = -10.0
            elif key.char.lower() == 'c':
                self.yaw_manual = 10.0
            elif key.char.lower() == 'x':
                self.yaw_manual = -10.0
        except AttributeError:
            pass  # handle special keys like shift, etc.

    def on_release(self, key):
        try:
            if key.char.lower() in ['w', 's']:
                self.x_manual = 0.0
            if key.char.lower() in ['a', 'd']:
                self.y_manual = 0.0
            if key.char.lower() in ['r', 'f']:
                self.z_manual = 0.0
            if key.char.lower() in ['c', 'x']:
                self.yaw_manual = 0.0

        except AttributeError:
            pass



    def publish_pcmd_thread_callback(self):
        while self.running:

            if self.flag == 'manual_control':

                self.pcmd_pub.x = self.x_manual
                self.pcmd_pub.y = self.y_manual
                self.pcmd_pub.z = self.z_manual
                self.pcmd_pub.yaw = self.yaw_manual
                
            elif self.flag == 'collect_data':

                self.pcmd_pub.x = self.pcmd_test.x
                self.pcmd_pub.y = self.pcmd_test.y
                self.pcmd_pub.z = self.pcmd_test.z
                self.pcmd_pub.yaw = self.pcmd_test.yaw
            
            else:
                rospy.logwarn("Failed to publish PCMD from Drone.")

            self.pcmd_publisher.publish(self.pcmd_pub)

            time.sleep(0.01)

        

    

    def subcribe_quadrotor_msg_callback(self, msg):
        index = msg.name.index("quadrotor::link")

        # Position
        pose = msg.pose[index]
        self.collect_drone_state.x = pose.position.x
        self.collect_drone_state.y = pose.position.y
        self.collect_drone_state.z = pose.position.z

        q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q)
        self.collect_drone_state.roll = roll
        self.collect_drone_state.pitch= pitch
        self.collect_drone_state.yaw = yaw

        twist = msg.twist[index]
        self.collect_drone_state.x_speed = twist.linear.x
        self.collect_drone_state.y_speed = twist.linear.y
        self.collect_drone_state.z_speed = twist.linear.z
        self.collect_drone_state.roll_speed = twist.angular.x
        self.collect_drone_state.pitch_speed = twist.angular.y
        self.collect_drone_state.yaw_speed = twist.angular.z




    def save_data_thread_callback(self):
        while self.running:
            next_time = time.perf_counter()
            if self.write_data_flag:
                # self.get_logger().info("Saving data...")
                formatted_time_stamp = f"{self.time_stamp:.2f}"
                data = [formatted_time_stamp,
                        self.collect_drone_state.roll, 
                        self.collect_drone_state.roll_speed, 
                        self.collect_drone_state.pitch, 
                        self.collect_drone_state.pitch_speed, 
                        self.collect_drone_state.z, 
                        self.collect_drone_state.z_speed, 
                        self.collect_drone_state.yaw, 
                        self.collect_drone_state.yaw_speed, 
                        self.pcmd_pub.x, 
                        self.pcmd_pub.y, 
                        self.pcmd_pub.z, 
                        self.pcmd_pub.yaw]
                
                with open(self.csv_file, mode='a', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow(data)
                self.time_stamp += 1 / self.freq

            next_time += 1 / self.freq
            sleep_time = next_time - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)





if __name__ == '__main__':
    node = CollectData()
    rospy.spin()
