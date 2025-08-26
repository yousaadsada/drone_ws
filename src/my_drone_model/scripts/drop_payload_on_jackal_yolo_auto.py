#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from pynput import keyboard
import math
import casadi as ca
import numpy as np
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Wrench
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
import threading
from tf.transformations import quaternion_matrix
from tf.transformations import quaternion_inverse
from tf.transformations import quaternion_multiply
from geometry_msgs.msg import Point
from my_custom_msgs.msg import PCMD, DroneState, PnPDataYolo
import time
import tkinter as tk
from gazebo_msgs.msg import ModelStates
from scipy.signal import place_poles
import csv
import os
from std_msgs.msg import Float64
from std_msgs.msg import Bool

class ManualRollControl:
    def __init__(self):
        rospy.init_node('manual_roll_control')

        self.running = True
        self.mode = 'mpc'

        self.px = [-1, -2]
        self.py = [-1, -2]
        self.pz = [-1, -2]

        self.R = np.eye(3)

        self.Ax = np.array([[0, 1],
                            [0, 0]])
        self.Bx = np.array([[0],
                            [1]])
        self.Ay = np.array([[0, 1],
                            [0, 0]])
        self.By = np.array([[0],
                            [1]])
        self.Az = np.array([[0, 1],
                            [0, 0]])
        self.Bz = np.array([[0],
                            [1]])
        
        self.Kx, self.Ky, self.Kz = self.calculate_K_xyz()

        self.task = "return"
        self.gripper_command = False
        self.is_collecting_on = False

        self.current_drone_state = DroneState()
        self.jackal_state = DroneState()
        self.jackal_state_yolo = DroneState()

        self.nx = 6
        self.nu = 4
        
        self.g = 9.8

        self.mpc_intervel = 0.02
        self.publish_intervel = 0.01
        self.predictive_horizon = 10
 
        self.a_z = 0.0
        self.a_yaw = 0.0
        self.b_z = 0.0
        self.b_yaw = 0.0
        self.a_roll = 0.0
        self.a_pitch = 0.0
        self.b_roll = 0.0
        self.b_pitch = 0.0


        csv_path = os.path.join(os.path.expanduser("~"), 'drone_ws', 'data', 'drone_state_function', 'state_function_matrix', 'state_function_matrix.csv')
        self.load_ab_parameters(csv_path)

        self.ref_x = 0.0
        self.ref_y = 0.0
        self.ref_z = 2.0
        self.ref_yaw = 0.0
        self.ref_x_speed = 0.0
        self.ref_y_speed = 0.0
        self.ref_z_speed = 0.0
        self.ref_yaw_speed = 0.0

        self.ref_camera_pitch = 0.5

        self.ref_roll = 0.0
        self.ref_pitch = 0.0

        self.time_stamp = 0.0

        self.pcmd_value = PCMD()

        self.jackal_command = Twist()

        self.data_dir = os.path.join(os.path.expanduser("~"), 'drone_ws', 'data', 'drop_payload_on_jackal')
        os.makedirs(self.data_dir, exist_ok=True) 
        self.csv_file = None

        self.pcmd_publisher = rospy.Publisher('/pcmd', PCMD, queue_size=1)
        self.ref_camera_pitch_publisher = rospy.Publisher('/ref_camera_pitch', Float64, queue_size=1)
        self.gripper_command_publisher = rospy.Publisher('/gripper_command', Bool, queue_size=1)
        self.jackal_command_publisher = rospy.Publisher('/jackal_command', Twist, queue_size=1)
        self.subscribe_quadrotor_msg = rospy.Subscriber('/gazebo/link_states_throttled', LinkStates, self.subcribe_quadrotor_msg_callback, queue_size=1)
        # self.subscribe_jackal_pos = rospy.Subscriber('/position_yolo', PnPDataYolo, self.subcribe_jackal_pos_callback, queue_size=1)       
        self.subscribe_jackal_pos_vel_kalman_filter = rospy.Subscriber('/pos_vel_kalman_filter', DroneState, self.subscribe_jackal_pos_vel_kalman_filter_callback, queue_size=1)
        self.calculate_ref_thread = threading.Thread(target=self.calculate_ref_thread_callback)
        self.calculate_ref_thread.daemon = True
        self.calculate_ref_thread.start()

        self.mpc_controller_init()
        self.do_mpc_thread = threading.Thread(target=self.do_mpc_thread_callback)
        self.do_mpc_thread.daemon = True
        self.do_mpc_thread.start()

        self.gui_thread = threading.Thread(target=self.run_gui)
        self.gui_thread.daemon = True
        self.gui_thread.start()

        self.collecting_data_thread = threading.Thread(target=self.collecting_data_thread_callback)
        self.collecting_data_thread.daemon = True
        self.collecting_data_thread.start()
        
        self.pub_jackal_command_thread = threading.Thread(target=self.pub_jackal_command_thread_callback)
        self.pub_jackal_command_thread.daemon = True
        self.pub_jackal_command_thread.start()

    def run_gui(self):
        self.root = tk.Tk()
        self.root.title("Dropping Payload")

        self.start_button = tk.Button(self.root, text="Start Dropping", command=self.start_dropping)
        self.start_button.grid(row=0, column=0)

        self.stop_button = tk.Button(self.root, text="returning", command=self.returning)
        self.stop_button.grid(row=0, column=1)
        self.root.mainloop()

    def start_dropping(self):
        self.task = "drop"
        self.is_collecting_on = True
        self.time_stamp = 0.0
        self.csv_file = os.path.join(self.data_dir, f'yolo_gazebo_pos_est_2.csv')
        self.write_csv_header() 

    def returning(self):
        self.task = "return"

    def write_csv_header(self):
        with open(self.csv_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['elapsed_time',
                             'jackal_x_gazebo', 'jackal_y_gazebo', 'jackal_z_gazebo', 
                             'jackal_x_speed_gazebo', 'jackal_y_speed_gazebo', 'jackal_z_speed_gazebo', 
                             'jackal_x_yolo_filted', 'jackal_y_yolo_filted', 'jackal_z_yolo_filted', 
                             'jackal_x_speed_yolo_filted', 'jackal_y_speed_yolo_filted', 'jackal_z_speed_yolo_filted',
                             'jackal_x_yolo_raw', 'jackal_y_yolo_raw', 'jackal_z_yolo_raw',
                             'jackal_x_speed_yolo_raw', 'jackal_y_speed_yolo_raw', 'jackal_z_speed_yolo_raw'])
            
    def collecting_data_thread_callback(self):
        while self.running:
            if self.is_collecting_on == True:

                data = [self.time_stamp,
                        self.current_drone_state.x,
                        self.current_drone_state.y,
                        self.current_drone_state.z,
                        self.current_drone_state.yaw,
                        self.current_drone_state.x_speed,
                        self.current_drone_state.y_speed,
                        self.current_drone_state.z_speed,
                        self.current_drone_state.yaw_speed,
                        self.jackal_state.x,
                        self.jackal_state.y,
                        self.jackal_state.z,
                        self.jackal_state.x_speed,
                        self.jackal_state.y_speed,
                        self.jackal_state.z_speed,
                        self.jackal_state_yolo.x,
                        self.jackal_state_yolo.y,
                        self.jackal_state_yolo.z,
                        self.jackal_state_yolo.x_speed,
                        self.jackal_state_yolo.y_speed,
                        self.jackal_state_yolo.z_speed,
                        ]

                with open(self.csv_file, mode='a', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow(data)
        
            time.sleep(0.02)



    def calculate_K_xyz(self):
        result_x = place_poles(self.Ax, self.Bx, self.px)
        Kx = result_x.gain_matrix
        result_y = place_poles(self.Ay, self.By, self.py)
        Ky = result_y.gain_matrix
        result_z = place_poles(self.Az, self.Bz, self.pz)
        Kz = result_z.gain_matrix

        return Kx, Ky, Kz


    def load_ab_parameters(self, csv_path):
        with open(csv_path, mode='r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                model = row['Model'].lower()  # e.g., 'roll'
                a = float(row['a'])
                b = float(row['b'])

                if model == 'roll':
                    self.a_roll = a
                    self.b_roll = b
                elif model == 'pitch':
                    self.a_pitch = a
                    self.b_pitch = b
                elif model == 'yaw':
                    self.a_yaw = a
                    self.b_yaw = b
                elif model == 'z':
                    self.a_z = a
                    self.b_z = b






    def subcribe_quadrotor_msg_callback(self, msg):

        index = msg.name.index("quadrotor::link")
        index_jackal = msg.name.index("jackal::base_link")

        # Position
        pose = msg.pose[index]
        self.current_drone_state.x = pose.position.x
        self.current_drone_state.y = pose.position.y
        self.current_drone_state.z = pose.position.z

        q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q)
        self.current_drone_state.roll = roll
        self.current_drone_state.pitch= pitch
        self.current_drone_state.yaw = yaw

        twist = msg.twist[index]
        self.current_drone_state.x_speed = twist.linear.x
        self.current_drone_state.y_speed = twist.linear.y
        self.current_drone_state.z_speed = twist.linear.z
        self.current_drone_state.roll_speed = twist.angular.x
        self.current_drone_state.pitch_speed = twist.angular.y
        self.current_drone_state.yaw_speed = twist.angular.z

        T = quaternion_matrix(q) 
        self.R = T[:3, :3]    
        self.R_torque = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                [np.sin(yaw), np.cos(yaw), 0],
                [0, 0, 1]])
        
        pose = msg.pose[index_jackal]
        self.jackal_state.x = pose.position.x
        self.jackal_state.y = pose.position.y
        self.jackal_state.z = pose.position.z + 0.15

        q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q)
        self.jackal_state.roll = roll
        self.jackal_state.pitch= pitch
        self.jackal_state.yaw = yaw

        twist = msg.twist[index_jackal]
        self.jackal_state.x_speed = twist.linear.x
        self.jackal_state.y_speed = twist.linear.y
        self.jackal_state.z_speed = twist.linear.z
        self.jackal_state.roll_speed = twist.angular.x
        self.jackal_state.pitch_speed = twist.angular.y
        self.jackal_state.yaw_speed = twist.angular.z



    def subscribe_jackal_pos_vel_kalman_filter_callback(self, msg):
        if msg.target == True:
            self.jackal_state_yolo.x = msg.x
            self.jackal_state_yolo.y = msg.y
            self.jackal_state_yolo.z = msg.z
            self.jackal_state_yolo.x_speed = msg.x_speed
            self.jackal_state_yolo.y_speed = msg.y_speed
            self.jackal_state_yolo.z_speed = msg.z_speed

            if abs(self.jackal_state_yolo.x - self.current_drone_state.x) < 0.1 and abs(self.jackal_state_yolo.y - self.current_drone_state.y) < 0.1:
                self.gripper_command = True
            else:
                pass
        
        else:
            pass

        if self.task == "return":
            self.ref_x = 0.0
            self.ref_y = 0.0
            self.ref_yaw = 0.0
            self.ref_x_speed = 0.0
            self.ref_y_speed = 0.0
            self.ref_yaw_speed = 0.0
            self.ref_camera_pitch = 0.5
        
        elif self.task == "drop":


            self.ref_x = msg.x
            self.ref_y = msg.y
            
            self.ref_x_speed = self.jackal_state_yolo.x_speed 
            self.ref_y_speed = self.jackal_state_yolo.y_speed
            dx = self.jackal_state_yolo.x - self.current_drone_state.x
            dy = self.jackal_state_yolo.y - self.current_drone_state.y

            panel_distance = math.sqrt(dx**2 + dy**2)
            vertical_distance = self.current_drone_state.z - self.jackal_state_yolo.z
            self.ref_camera_pitch = math.atan2(vertical_distance, panel_distance)

            if self.ref_camera_pitch < 0.8 * math.pi / 2:
                self.ref_yaw = math.atan2(dy, dx)   

            else:
                pass

        else:
            pass

        self.ref_camera_pitch_publisher.publish(self.ref_camera_pitch)
        self.gripper_command_publisher.publish(self.gripper_command)
        

    def pub_jackal_command_thread_callback(self):
        while self.running:
            start_time = rospy.get_time()
            if self.is_collecting_on == True:
                self.jackal_command.linear.x = 0.6
                self.jackal_command.angular.z = 0.0
                self.jackal_command_publisher.publish(self.jackal_command)
                self.time_stamp += 0.01
            
            else:
                pass
                        
            current_time = rospy.get_time()
            delta_time = current_time - start_time
            rospy.sleep(0.01 - delta_time)


    def calculate_ref_thread_callback(self):

        while self.running:
            
            ex = self.ref_x - self.current_drone_state.x
            if ex > 1.0:
                ex = 1.0
            elif ex < -1.0:
                ex = -1.0
            ex_dot = self.ref_x_speed - self.current_drone_state.x_speed

            ey = self.ref_y - self.current_drone_state.y
            if ey > 1.0:
                ey = 1.0
            elif ey < -1.0:
                ey = -1.0
            ey_dot = self.ref_y_speed - self.current_drone_state.y_speed

            ez = self.ref_z - self.current_drone_state.z
            ez_dot = self.ref_z_speed - self.current_drone_state.z_speed

            state_error_x = np.array([[ex],
                                      [ex_dot]])
            state_error_y = np.array([[ey],
                                      [ey_dot]])
            state_error_z = np.array([[ez],
                                      [ez_dot]])

            ux = -self.Kx @ state_error_x
            uy = -self.Ky @ state_error_y
            uz = -self.Kz @ state_error_z

            vx = ux
            vy = uy
            vz = uz

            Psi_current = self.current_drone_state.yaw
            
            a = vx / (vz + self.g)
            b = vy / (vz + self.g)
            c = math.cos(Psi_current)
            d = math.sin(Psi_current)

            tan_theta = a * c + b * d
            Theta_ref = math.atan (tan_theta)

            tan_phi = -a * d + b * c
            Phi_ref = math.atan(tan_phi)

            self.Uz = vz

            self.ref_roll = Phi_ref
            self.ref_pitch = -Theta_ref

            rospy.loginfo_throttle(0.2, f"vx={vx}, vy={vy}, pitch_ref={Theta_ref:.3f}, roll_ref={Phi_ref:.3f}, ex={ex:.3f}, ex_dot={ex_dot:.3f}")

            time.sleep(0.01)



    def mpc_controller_init(self):

        t = self.mpc_intervel 
        h = self.predictive_horizon 

        a_z = self.a_z
        a_yaw = self.a_yaw
        b_z = self.b_z
        b_yaw = self.b_yaw

        a_roll = self.a_roll
        a_pitch = self.a_pitch
        b_roll = self.b_roll
        b_pitch = self.b_pitch

        z = ca.SX.sym('z')
        roll = ca.SX.sym('roll')
        pitch = ca.SX.sym('pitch')
        yaw = ca.SX.sym('yaw')
        v_z = ca.SX.sym('v_z')
        v_yaw = ca.SX.sym('v_yaw')
        
        z_input = ca.SX.sym('z_input')  
        yaw_input = ca.SX.sym('yaw_input')  
        roll_input = ca.SX.sym('roll_input')  
        pitch_input = ca.SX.sym('pitch_input')  

        states = ca.vertcat(roll, pitch, yaw, z, v_yaw, v_z)
        controls = ca.vertcat(roll_input, pitch_input, yaw_input, z_input)

        # Define the system dynamics
        next_states = ca.vertcat(
            a_roll * roll + b_roll * roll_input,
            a_pitch * pitch + b_pitch * pitch_input,
            yaw + t * v_yaw,
            z + t * v_z,
            a_yaw * v_yaw + b_yaw * yaw_input,
            a_z * v_z + b_z * z_input
        )
    
        f = ca.Function('f', [states, controls], [next_states])

        # Optimization variables
        U = ca.SX.sym('U', 4, h)  # Control inputs over the horizon (v, w)
        X = ca.SX.sym('X', 6, h + 1)  # State variables over the horizon (x, y, theta)
 

        # roll_input_min = -20
        # roll_input_max = 20
        # pitch_input_min = -20
        # pitch_input_max = 20
        # yaw_input_min = -20
        # yaw_input_max = 20
        # z_input_min = -20
        # z_input_max = 20


        roll_input_min = -10
        roll_input_max = 10
        pitch_input_min = -10
        pitch_input_max = 10
        yaw_input_min = -10
        yaw_input_max = 10
        z_input_min = -10
        z_input_max = 10


        self.lbx = np.concatenate(
            [np.full(self.nx * (h + 1), -ca.inf), np.tile([roll_input_min, pitch_input_min, yaw_input_min, z_input_min], h)]
        )
        self.ubx = np.concatenate(
            [np.full(self.nx * (h + 1), ca.inf), np.tile([roll_input_max, pitch_input_max, yaw_input_max, z_input_max], h)]
        )
        
        cost_fn = 0
        g = []

        P = ca.SX.sym('P', self.nx + self.nu)

        g.append(X[:,0] - P[:self.nx])

        Q = np.eye(4)
        S = np.eye(4) * 0.001
        R = np.eye(4) * 0.000001
        R[2, 2] = 0.0001
        R[-1, -1] = 0.0001

        # Loop over the prediction horizon
        for k in range(h):
            st = X[:, k]
            state = st[:4]
            con = U[:, k]
            x_ref = P[-4:]

            if k == h-1:
                cost_fn += (state - x_ref).T @ Q @ (state - x_ref)
            else:
                cost_fn += (state - x_ref).T @ S @ (state - x_ref)

            cost_fn += con.T @ R @ con
    
            st_next = X[:, k+1]
            f_value = f(st, con)
            g.append(st_next - f_value)  # Dynamics constraint


        # Concatenate constraints and optimization variables
        g = ca.vertcat(*g)
        OPT_variables = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))

        # Define optimization problem
        nlp_prob = {
            'f':cost_fn,
            'x':OPT_variables,
            'g':g,
            'p':P
        }

        opts = {
            'ipopt.max_iter':1000,
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.tol': 1e-6
        }

        # Create solver
        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

    def do_mpc_thread_callback(self):
        def get_R(yaw):
            R = np.array([
                [np.cos(yaw), -np.sin(yaw)],
                [np.sin(yaw), np.cos(yaw)]
            ])
            return R

        while self.running:

            if self.mode == 'mpc':
                h = self.predictive_horizon
                n_states = self.nx
                n_controls = self.nu

                self.R_inv = np.linalg.inv(self.R)
                A_w = np.array([self.current_drone_state.roll_speed, self.current_drone_state.pitch_speed, self.current_drone_state.yaw_speed]) 
                A_b = self.R_inv.dot(A_w)
                roll_speed, pitch_speed, yaw_speed = A_b

                x_current_state = np.array([self.current_drone_state.roll,
                                            self.current_drone_state.pitch, 
                                            self.current_drone_state.yaw, 
                                            self.current_drone_state.z,
                                            yaw_speed,
                                            self.current_drone_state.z_speed])
                
                x_ref = np.array([self.ref_roll, 
                                  self.ref_pitch, 
                                  self.ref_yaw,
                                  self.ref_z])

                u0 = np.zeros((n_controls * h, 1 ))
                u0 = u0.flatten()
                x_init = np.tile(x_current_state, (h + 1, 1)).T.flatten()
                P = np.concatenate((x_current_state, x_ref))
                
                args = {
                    'x0': np.concatenate([x_init, u0]),  # Initial guess for states and controls
                    'lbx': self.lbx,
                    'ubx': self.ubx,
                    'lbg': np.zeros((n_states * (h + 1),)),  # Lower bounds on constraints
                    'ubg': np.zeros((n_states * (h + 1),)),  # Upper bounds on constraints
                    'p': P  # Pass the current state and reference as parameters
                }

                sol = self.solver(**args)

                u_opt = sol['x'][n_states * (h + 1):].full().reshape((h, n_controls))


                u = np.zeros(4)
                u[0] = u_opt[0, 0] 
                u[1] = u_opt[0, 1]
                u[2] = u_opt[0, 2]
                u[3] = u_opt[0, 3]


                self.pcmd_value.y = u[0]
                self.pcmd_value.x = u[1]
                self.pcmd_value.yaw = u[2]
                self.pcmd_value.z = u[3]

                self.pcmd_publisher.publish(self.pcmd_value)

            time.sleep(0.01)




if __name__ == '__main__':
    node = ManualRollControl()
    rospy.spin()
