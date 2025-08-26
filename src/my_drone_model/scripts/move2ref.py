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
from my_custom_msgs.msg import PCMD, DroneState
import time
import tkinter as tk
from gazebo_msgs.msg import ModelStates
from scipy.signal import place_poles
import csv
import os

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

        self.roll_pos_ref = 0.0  # In degrees
        self.pitch_pos_ref = 0.0
        self.yaw_speed_ref = 0.0
        self.z_speed_ref = 0.0
        self.z_pos_ref = 0.0

        self.current_drone_state = DroneState()

        self.nx = 6
        self.nu = 4
        
        self.g = 9.8

        self.mpc_intervel = 0.02
        self.publish_intervel = 0.01
        self.predictive_horizon = 10

        self.pcmd_publisher = rospy.Publisher('/pcmd', PCMD, queue_size=1)
        self.ref_pitch_publisher = rospy.Publisher('/ref_pitch', Float32, queue_size=1)
        self.subcribe_quadrotor_msg = rospy.Subscriber('/gazebo/link_states_throttled', LinkStates, self.subcribe_quadrotor_msg_callback, queue_size=1)
        
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
        self.ref_z = 1.0
        self.ref_yaw = 0.0
        self.ref_x_speed = 0.0
        self.ref_y_speed = 0.0
        self.ref_z_speed = 0.0
        self.ref_yaw_speed = 0.0

        self.ref_roll = 0.0
        self.ref_pitch = 0.0

        self.pcmd_value = PCMD()

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


    def run_gui(self):
        self.root = tk.Tk()
        self.root.title("Reference State Input")

        self.entries = []
        self.labels = [
            "x", "y", "z", "yaw",
            "x_speed", "y_speed", "z_speed", "yaw_speed"
        ]

        for i, label in enumerate(self.labels):
            tk.Label(self.root, text=label).grid(row=i, column=0)
            entry = tk.Entry(self.root)
            entry.grid(row=i, column=1)
            self.entries.append(entry)

        self.send_button = tk.Button(self.root, text="Send", command=self.send_data)
        self.send_button.grid(row=8, columnspan=2)

        self.root.mainloop()

    def send_data(self):
        try:
            self.ref_x = float(self.entries[0].get())
            self.ref_y = float(self.entries[1].get())
            self.ref_z = float(self.entries[2].get())
            self.ref_yaw = float(self.entries[3].get())
            self.ref_x_speed = float(self.entries[4].get())
            self.ref_y_speed = float(self.entries[5].get())
            self.ref_z_speed = float(self.entries[6].get())
            self.ref_yaw_speed = float(self.entries[7].get())
        except ValueError:
            rospy.logwarn("Invalid input detected. Using default 0.0 values.")
            self.ref_x = self.ref_y = self.ref_z = self.ref_yaw = 0.0
            self.ref_x_speed = self.ref_y_speed = self.ref_z_speed = self.ref_yaw_speed = 0.0




    def subcribe_quadrotor_msg_callback(self, msg):

        index = msg.name.index("quadrotor::link")

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
        
        
            
            # rospy.loginfo(self.R)


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
            
            # a = vx / (vz + self.g)
            # b = vy / (vz + self.g)
            a = vx / (self.g)
            b = vy / (self.g)
            c = math.cos(Psi_current)
            d = math.sin(Psi_current)

            tan_theta = a * c + b * d
            Theta_ref = math.atan (tan_theta)

            tan_phi = -a * d + b * c
            Phi_ref = math.atan(tan_phi)

            self.Uz = vz

            self.ref_roll = Phi_ref
            self.ref_pitch = -Theta_ref

            self.ref_pitch_publisher.publish(self.ref_pitch)

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
        # R = np.eye(4) * 0.0001
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
                
                ref_yaw_revised = (self.ref_yaw + math.pi) % (2 * math.pi) - math.pi
                if ref_yaw_revised - self.current_drone_state.yaw < -math.pi:
                    ref_yaw_revised += 2 * math.pi
                elif ref_yaw_revised - self.current_drone_state.yaw > math.pi:
                    ref_yaw_revised += -2 * math.pi

                x_ref = np.array([self.ref_roll, 
                                  self.ref_pitch, 
                                  ref_yaw_revised,
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
