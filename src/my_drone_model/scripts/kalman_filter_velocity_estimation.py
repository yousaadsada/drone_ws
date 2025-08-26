import rospy
from my_custom_msgs.msg import DroneState, PnPDataYolo
import numpy as np
from std_msgs.msg import Float64

class VelocityEstimation:
    def __init__(self):
        rospy.init_node('velocity_estimation')
        self.running = True
        
        self.jackal_state = DroneState()
        self.jackal_state_predictive = DroneState()

        self.current_time_speed_updata = None
        self.kalman_filter_flag = False
        self.dt = None

        self.jackal_position_buffer_x = []
        self.jackal_position_buffer_y = []
        self.jackal_position_buffer_z = []

        self.jackal_velocity_buffer_x = []
        self.jackal_velocity_buffer_y = []
        self.jackal_velocity_buffer_z = []


        self.previous_time_jackal_velocity = None
        self.jackal_previous_x = 0.0
        self.jackal_previous_y = 0.0
        self.jackal_previous_z = 0.0

        delta_t = 1 / 20

        # self.x = np.zeros((6,1))
        

        # self.A_K = np.array([
        #           [1,0,0,delta_t,0,0],
        #           [0,1,0,0,delta_t,0],
        #           [0,0,1,0,0,delta_t],
        #           [0,0,0,1,0,0],
        #           [0,0,0,0,1,0],
        #           [0,0,0,0,0,1]
        # ])

        # self.H = np.array([
        #           [1,0,0,0,0,0],
        #           [0,1,0,0,0,0],
        #           [0,0,1,0,0,0]
        # ])
        
        # process_noise = 1.0
        # measurement_noise = 1.0
        # self.Q = np.eye(6) * process_noise  
        # self.R = np.eye(3) * measurement_noise
        # self.S = np.eye(6)

        self.x = np.zeros((9,1))

        self.A_K = np.array([
            [1,0,0,delta_t,0,0,delta_t**2/2,0,0],
            [0,1,0,0,delta_t,0,0,delta_t**2/2,0],
            [0,0,1,0,0,delta_t,0,0,delta_t**2/2],
            [0,0,0,1,0,0,delta_t,0,0],
            [0,0,0,0,1,0,0,delta_t,0],
            [0,0,0,0,0,1,0,0,delta_t],
            [0,0,0,0,0,0,1,0,0],
            [0,0,0,0,0,0,0,1,0],
            [0,0,0,0,0,0,0,0,1],
        ])

        self.H = np.array([
                  [1,0,0,0,0,0,0,0,0],
                  [0,1,0,0,0,0,0,0,0],
                  [0,0,1,0,0,0,0,0,0]
        ])

        sigma_a = 5.0
        sigma_pos = 0.5
        self.Q = np.zeros((9, 9))
        self.Q[0:3, 0:3] = np.diag([delta_t**4/4, delta_t**4/4, delta_t**4/4]) * sigma_a**2
        self.Q[0:3, 3:6] = np.diag([delta_t**3/2, delta_t**3/2, delta_t**3/2]) * sigma_a**2
        self.Q[0:3, 6:9] = np.diag([delta_t**2/2, delta_t**2/2, delta_t**2/2]) * sigma_a**2
        self.Q[3:6, 0:3] = self.Q[0:3, 3:6].T
        self.Q[3:6, 3:6] = np.diag([delta_t**2, delta_t**2, delta_t**2]) * sigma_a**2
        self.Q[3:6, 6:9] = np.diag([delta_t, delta_t, delta_t]) * sigma_a**2
        self.Q[6:9, 0:3] = self.Q[0:3, 6:9].T
        self.Q[6:9, 3:6] = self.Q[3:6, 6:9].T
        self.Q[6:9, 6:9] = np.eye(3) * sigma_a**2

        self.R = np.eye(3) * sigma_pos**2

        self.S = np.eye(9)

        self.subcribe_jackal_pos = rospy.Subscriber('/position_yolo', PnPDataYolo, self.jackal_pos_callback, queue_size=1)
        self.publish_jackal_pos_vel_kalman_filter = rospy.Publisher('/pos_vel_kalman_filter', DroneState, queue_size=1)


    def jackal_pos_callback(self, msg):

        def kf_predict():
            self.x = self.A_K @ self.x
            self.S = self.A_K @ self.S @ self.A_K.T + self.Q

        def kf_update(measured_position):
            Z = np.array(measured_position).reshape(3, 1)
            K = self.S @ self.H.T @ np.linalg.inv(self.H @ self.S @ self.H.T + self.R)
            self.x = self.x + K @ (Z - self.H @ self.x)
            # I = np.eye(6)
            I = np.eye(9)
            self.S = (I - K @ self.H) @ self.S        


        if msg.target == False:
            self.jackal_state.target = False
            self.jackal_state_predictive.target = False
        
        elif msg.target == True:
            self.jackal_state.target = True
            self.jackal_state_predictive.target = True
            self.jackal_state.x = msg.tx
            self.jackal_state.y = msg.ty
            self.jackal_state.z = msg.tz
       
        current_time = rospy.Time.now().to_sec()

        if self.previous_time_jackal_velocity is not None:
            delta_time = current_time - self.previous_time_jackal_velocity
            delta_x = self.jackal_state.x - self.jackal_previous_x
            delta_y = self.jackal_state.y - self.jackal_previous_y
            delta_z = self.jackal_state.z - self.jackal_previous_z
            self.jackal_state.x_speed = delta_x / delta_time
            self.jackal_state.y_speed = delta_y / delta_time
            self.jackal_state.z_speed = delta_z / delta_time

        
        else:
            self.jackal_state.x_speed = 0.0
            self.jackal_state.y_speed = 0.0
            self.jackal_state.z_speed = 0.0

        self.previous_time_jackal_velocity = current_time
        self.jackal_previous_x = self.jackal_state.x
        self.jackal_previous_y = self.jackal_state.y
        self.jackal_previous_z = self.jackal_state.z

        
        if self.kalman_filter_flag == False:
            self.x = np.array([[self.jackal_state.x],
                               [self.jackal_state.y],
                               [self.jackal_state.z],
                               [0.0],
                               [0.0],
                               [0.0],
                               [0.0],
                               [0.0],
                               [0.0]])  

            self.jackal_state_predictive.x = self.jackal_state.x
            self.jackal_state_predictive.y = self.jackal_state.y
            self.jackal_state_predictive.z = self.jackal_state.z
            self.jackal_state_predictive.x_speed = 0.0
            self.jackal_state_predictive.y_speed = 0.0
            self.jackal_state_predictive.z_speed = 0.0
            
            self.kalman_filter_flag = True

 
        
        else:
            kf_predict()
            measured_position = [self.jackal_state.x, self.jackal_state.y, self.jackal_state.z]
            kf_update(measured_position)


            self.jackal_state_predictive.x = self.x[0,0]
            self.jackal_state_predictive.y = self.x[1,0]
            self.jackal_state_predictive.z = self.x[2,0]
            self.jackal_state_predictive.x_speed = self.x[3,0]
            self.jackal_state_predictive.y_speed = self.x[4,0]
            self.jackal_state_predictive.z_speed = self.x[5,0]

        self.publish_jackal_pos_vel_kalman_filter.publish(self.jackal_state_predictive)

if __name__ == '__main__':
    node = VelocityEstimation()
    rospy.spin()