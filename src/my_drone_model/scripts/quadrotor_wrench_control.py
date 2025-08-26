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
import time
from my_custom_msgs.msg import PCMD, DroneState
from gazebo_msgs.msg import ModelStates
import tkinter as tk


class ManualRollControl:
    def __init__(self):
        rospy.init_node('manual_roll_control')

        self.running = True
        self.take_off_enabled = False
        self.mode = 'mpc'

        self.roll_pos_ref = 0.0  # In degrees
        self.pitch_pos_ref = 0.0
        self.yaw_speed_ref = 0.0
        self.z_speed_ref = 0.0
   
        self.current_drone_state = DroneState()

        self.R = np.eye(3)
        self.R_torque = np.eye(3)

        self.tx_body = 0.0
        self.ty_body = 0.0
        self.tz_body = 0.0
        self.thrust_body = 0.0 

        self.tx_world = 0.0
        self.ty_world = 0.0
        self.tz_world = 0.0
        self.thrust_world = 0.0 

        self.nx = 6
        self.nu = 4
        
        self.mass = 1.04
        self.ixx = 0.005
        self.iyy = 0.005
        self.izz = 0.009
        self.gravity = 9.8

        self.euler_angle = Point()
        self.euler_angle_speed = Point()
        self.input = Point()

        self.mpc_intervel = 0.01
        self.publish_intervel = 0.01
        self.predictive_horizon = 20

        self.subcribe_quadrotor_msg = rospy.Subscriber('/gazebo/link_states_throttled', LinkStates,  self.subcribe_quadrotor_msg_callback, queue_size=1)
        self.subcribe_pcmd = rospy.Subscriber('/pcmd', PCMD, self.subcribe_pcmd_callback, queue_size=1)
        self.publish_wrench = rospy.Publisher('/force', Wrench, queue_size=1)


        self.mpc_controller_init()
        self.do_mpc_thread = threading.Thread(target=self.do_mpc_thread_callback)
        self.do_mpc_thread.daemon = True
        self.do_mpc_thread.start()

        self.publish_wrench_thread = threading.Thread(target=self.publish_wrench_thread_callback)
        self.publish_wrench_thread.daemon = True
        self.publish_wrench_thread.start()

        self.gui_thread = threading.Thread(target=self.create_gui)
        self.gui_thread.daemon = True
        self.gui_thread.start()

    def create_gui(self):
        """Create a simple Tkinter window with a Take Off button."""
        self.root = tk.Tk()
        self.root.title("Quadrotor Control")
        self.root.geometry("200x100")
        
        take_off_button = tk.Button(self.root, text="Take Off", command=self.enable_take_off)
        take_off_button.pack(pady=20)
        
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.mainloop()

    def enable_take_off(self):
        """Set flag to enable wrench publishing."""
        self.take_off_enabled = True
        rospy.loginfo("Take Off button clicked, enabling wrench publishing.")

    def on_closing(self):
        """Handle window closing."""
        self.running = False
        self.root.destroy()

   

    def subcribe_pcmd_callback(self, msg):
        self.roll_pos_ref = -msg.y * math.pi / 180
        self.pitch_pos_ref = msg.x * math.pi / 180
        self.z_speed_ref = msg.z * 0.1
        self.yaw_speed_ref = msg.yaw * 0.1




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
            


    def mpc_controller_init(self):

        dt = self.mpc_intervel 
        h = self.predictive_horizon 

        mass = self.mass
        ixx = self.ixx
        iyy = self.iyy
        izz = self.izz
        gravity = self.gravity

        roll = ca.SX.sym('roll')
        pitch = ca.SX.sym('pitch')
        roll_speed = ca.SX.sym('roll_speed')
        pitch_speed = ca.SX.sym('pitch_speed')
        yaw_speed = ca.SX.sym('yaw_speed')
        z_speed = ca.SX.sym('z_speed')
        

        roll_input = ca.SX.sym('roll_input')  
        pitch_input = ca.SX.sym('pitch_input')  
        yaw_input = ca.SX.sym('yaw_input')  
        z_input = ca.SX.sym('z_input')  

        states = ca.vertcat(roll, pitch, roll_speed, pitch_speed, yaw_speed, z_speed)
        controls = ca.vertcat(roll_input, pitch_input, yaw_input, z_input)

        # Define the system dynamics
        # next_states = ca.vertcat(
        #     roll + roll_speed * dt,
        #     pitch + pitch_speed * dt,
        #     roll_speed + (roll_input/ixx) * dt,
        #     pitch_speed + (pitch_input/iyy) * dt,
        #     yaw_speed + (yaw_input/izz) * dt,
        #     z_speed + (z_input*ca.cos(roll)*ca.cos(pitch)/mass - gravity) * dt
        # )

        next_states = ca.vertcat(
            roll + roll_speed * dt,
            pitch + pitch_speed * dt,
            roll_speed + (roll_input/ixx) * dt,
            pitch_speed + (pitch_input/iyy) * dt,
            yaw_speed + (yaw_input/izz) * dt,
            z_speed + (z_input/mass - gravity) * dt
        )

    
        f = ca.Function('f', [states, controls], [next_states])

        # Optimization variables
        U = ca.SX.sym('U', 4, h)  # Control inputs over the horizon (v, w)
        X = ca.SX.sym('X', 6, h + 1)  # State variables over the horizon (x, y, theta)
 

        roll_input_min = -0.1
        roll_input_max = 0.1
        pitch_input_min = -0.1
        pitch_input_max = 0.1
        yaw_input_min = -0.1
        yaw_input_max = 0.1
        z_input_min = 5.0
        z_input_max = 15.0


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
        S = np.eye(4) * 0.0

        # Loop over the prediction horizon
        for k in range(h):
            st = X[:, k]
            state = ca.vertcat(st[0], st[1], st[4], st[5])
            con = U[:, k]
            x_ref = P[-4:]

            if k == h-1:
                cost_fn += (state - x_ref).T @ Q @ (state - x_ref)
            else:
                cost_fn += (state - x_ref).T @ S @ (state - x_ref)
    
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


        while self.running:

            start_time = time.time()

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
                            roll_speed, 
                            pitch_speed,
                            yaw_speed,
                            self.current_drone_state.z_speed])
    
                x_ref = np.array([self.roll_pos_ref, 
                                  self.pitch_pos_ref,
                                  self.yaw_speed_ref, 
                                  self.z_speed_ref])

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

                self.tx_body = u[0]
                self.ty_body = u[1]
                self.tz_body = u[2]
                self.thrust_body = u[3] 



    def publish_wrench_thread_callback(self):
        rate = rospy.Rate(1/self.publish_intervel)
        while self.running:
            if self.take_off_enabled:
                F_b = np.array([0.0, 0.0, self.thrust_body])
                T_b = np.array([self.tx_body, self.ty_body, self.tz_body]) 
                F_w = self.R.dot(F_b)
                T_w = self.R.dot(T_b)

                w = Wrench()
                w.force.x, w.force.y, w.force.z  = F_w
                w.torque.x, w.torque.y, w.torque.z = T_w

                self.publish_wrench.publish(w)
            
            else:
                rospy.loginfo("Waiting for Take Off button to be clicked.")

            rate.sleep()



if __name__ == '__main__':
    node = ManualRollControl()
    rospy.spin()
