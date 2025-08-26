import rospy
import numpy as np
import cv2, time
from my_custom_msgs.msg import KeyPoints, KpYolo, PnPDataYolo, JackalSize, DroneState
import threading
from gazebo_msgs.msg import LinkStates
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_matrix
from geometry_msgs.msg import Point


class PnpNode:
    def __init__(self):
        rospy.init_node("pnp_node")

        self.running = True

        self.previous_time = None
        self.dt = None
        
        self.kp_data = []
        self.TargetPoints = np.zeros((8,2), dtype=np.float32)
        self.target_position_camera_yolo = PnPDataYolo()
        self.target_position_world_yolo = PnPDataYolo()
        self.target_position_world_gazebo = PnPDataYolo()
        self.camera_state = DroneState()
        self.jackal_state_yolo = DroneState()
        self.jackal_state_gazebo = DroneState()

        self.previous_target_position_x = None
        self.previous_target_position_y = None
        self.previous_target_position_z = None
        #SOLVEPNP PARAMETERS

        self.jackal_size = JackalSize()

        self.jackal_size.length = 0.4
        self.jackal_size.width = 0.5
        self.jackal_size.height = 0.4

        self.drone_points = np.array([
            [ self.jackal_size.width/2,   self.jackal_size.length/2,   self.jackal_size.height/2],   # front_top_left
            [ self.jackal_size.width/2,  -self.jackal_size.length/2,   self.jackal_size.height/2],   # front_top_right
            [ self.jackal_size.width/2,   self.jackal_size.length/2,  -self.jackal_size.height/2],   # front_bottom_left
            [ self.jackal_size.width/2,  -self.jackal_size.length/2,  -self.jackal_size.height/2],   # front_bottom_right
            [-self.jackal_size.width/2,   self.jackal_size.length/2,   self.jackal_size.height/2],   # rear_top_left
            [-self.jackal_size.width/2,  -self.jackal_size.length/2,   self.jackal_size.height/2],   # rear_top_right
            [-self.jackal_size.width/2,   self.jackal_size.length/2,  -self.jackal_size.height/2],   # rear_bottom_left
            [-self.jackal_size.width/2,  -self.jackal_size.length/2,  -self.jackal_size.height/2]    # rear_bottom_right
        ], dtype=np.float32)

        
        self.camera_matrix = np.array([
                            [381.04, 0.0, 320.0],
                            [0.0, 381.04, 240.0],
                            [0.0, 0.0, 1.0]
                        ],dtype=np.float32)
        
        # self.camera_matrix = np.array([
        #             [775.19, 0.0, 640.0],
        #             [0.0, 775.19, 360.0],
        #             [0.0, 0.0, 1.0]
        #         ],dtype=np.float32)
        
        self.T_wc = np.zeros((4, 4)) 
        self.jackal_cam = np.array([0.0, 0.0, 0.0, 1.0]) 

        self.position_yolo_pub_camera = rospy.Publisher('/position_yolo_camera', PnPDataYolo, queue_size=1)
        self.position_yolo_pub = rospy.Publisher('/position_yolo', PnPDataYolo, queue_size=1)
        self.position_gazebo_pub = rospy.Publisher('/position_gazebo', PnPDataYolo, queue_size=1)
        self.jackal_pos_gazebo_pub = rospy.Publisher('/jackal_pos_gazebo', DroneState, queue_size=1)
        self.jackal_pos_yolo_pub = rospy.Publisher('/jackal_pos_yolo', DroneState, queue_size=1)
        self.targets_sub = rospy.Subscriber('/yolo_keypoints', KpYolo, self.kp_callback, queue_size=1)
        self.subcribe_quadrotor_msg = rospy.Subscriber('/gazebo/link_states_throttled', LinkStates, self.subcribe_quadrotor_msg_callback, queue_size=1)

        # self.get_jackal_pos_thread = threading.Thread(target = self.get_jackal_pos_thread_callback)
        # self.get_jackal_pos_thread.daemon = True
        # self.get_jackal_pos_thread.start()


    def subcribe_quadrotor_msg_callback(self, msg):

        index_drone = msg.name.index("quadrotor::camera_link")
        index_jackal = msg.name.index("jackal::base_link")
        # Position
        pose = msg.pose[index_drone]
        self.camera_state.x = pose.position.x
        self.camera_state.y = pose.position.y
        self.camera_state.z = pose.position.z

        q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q)
        self.camera_state.roll = roll
        self.camera_state.pitch= pitch
        self.camera_state.yaw = yaw

        twist = msg.twist[index_drone]
        self.camera_state.x_speed = twist.linear.x
        self.camera_state.y_speed = twist.linear.y
        self.camera_state.z_speed = twist.linear.z
        self.camera_state.roll_speed = twist.angular.x
        self.camera_state.pitch_speed = twist.angular.y
        self.camera_state.yaw_speed = twist.angular.z

        t_wc = np.array([self.camera_state.x,
                        self.camera_state.y,
                        self.camera_state.z])

        # Camera orientation quaternion (x, y, z, w)
        q = [pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w]

        # 4x4 homogeneous transform from camera to world
        T_wc = quaternion_matrix(q)
        T_wc[0:3, 3] = t_wc
        self.T_wc = T_wc

        pose = msg.pose[index_jackal]
        self.target_position_world_gazebo.target = True
        self.target_position_world_gazebo.tx = pose.position.x
        self.target_position_world_gazebo.ty = pose.position.y
        self.target_position_world_gazebo.tz = pose.position.z + 0.15

        self.jackal_state_gazebo.x = pose.position.x
        self.jackal_state_gazebo.y = pose.position.y
        self.jackal_state_gazebo.z = pose.position.z

        twist = msg.twist[index_jackal]

        self.jackal_state_gazebo.x_speed = twist.linear.x
        self.jackal_state_gazebo.y_speed = twist.linear.y
        self.jackal_state_gazebo.z_speed = twist.linear.z
                
   
    def kp_callback(self, msg):

        def estimate_3d_position(keypoints_flat):

            # 2. Reshape keypoints_flat into Nx2 array for image points
            if len(keypoints_flat) != 16:
                raise ValueError("Expected 16 values (8 keypoints with x, y), got: {}".format(len(keypoints_flat)))
            image_points = np.array(keypoints_flat).reshape(-1, 2).astype(np.float32)

            # 3. Solve PnP to get rotation and translation vectors
            success, rvec, tvec = cv2.solvePnP(
                self.drone_points,       # 3D model points
                image_points,       # Corresponding 2D image points
                self.camera_matrix,      # Camera intrinsic matrix
                None,        # Distortion coefficients
                flags=cv2.SOLVEPNP_ITERATIVE
            )

            if not success:
                raise RuntimeError("PnP solution failed.")

            return rvec, tvec

        try:
            target_ok = msg.target
            keypoints_flat = msg.keypoints.data 
            
            if target_ok:
                rvec, tvec = estimate_3d_position(keypoints_flat)

                tx, ty, tz = tvec.flatten()

                self.target_position_camera_yolo.target = True
                # self.target_position_camera_yolo.tx = tx
                # self.target_position_camera_yolo.ty = ty
                # self.target_position_camera_yolo.tz = tz
                self.target_position_camera_yolo.tx = tz
                self.target_position_camera_yolo.ty = -tx
                self.target_position_camera_yolo.tz = -ty

                self.jackal_cam = np.array([tz, -tx, -ty, 1.0]) 

            elif not target_ok:
                self.target_position_camera_yolo.target = False
                self.target_position_camera_yolo.tx = 0.0
                self.target_position_camera_yolo.ty = 0.0
                self.target_position_camera_yolo.tz = 0.0

            if self.target_position_camera_yolo.target == True:
                jackal_world = self.T_wc @ self.jackal_cam
                jackal_world = jackal_world[0:3]

                jackal_x_world = jackal_world[0]
                jackal_y_world = jackal_world[1]
                jackal_z_world = jackal_world[2]

                self.target_position_world_yolo.target = True
                self.target_position_world_yolo.tx = jackal_x_world
                self.target_position_world_yolo.ty = jackal_y_world
                self.target_position_world_yolo.tz = jackal_z_world

                self.jackal_state_yolo.target = True
            
            else:
                self.target_position_world_yolo.target = False
                # self.target_position_world_yolo.tx = 0.0
                # self.target_position_world_yolo.ty = 0.0
                # self.target_position_world_yolo.tz = 0.0

                self.jackal_state_yolo.target = False
            
            current_time = rospy.Time.now().to_sec()
            if self.previous_time is not None:
                self.dt = current_time - self.previous_time
            self.previous_time = current_time

            if self.dt is not None:
                self.jackal_state_yolo.x_speed = (self.target_position_world_yolo.tx - self.previous_target_position_x) / self.dt
                self.jackal_state_yolo.y_speed = (self.target_position_world_yolo.ty - self.previous_target_position_y) / self.dt
                self.jackal_state_yolo.z_speed = (self.target_position_world_yolo.tz - self.previous_target_position_z) / self.dt
            
            self.jackal_state_yolo.x = self.target_position_world_yolo.tx
            self.jackal_state_yolo.y = self.target_position_world_yolo.ty
            self.jackal_state_yolo.z = self.target_position_world_yolo.tz

            self.previous_target_position_x = self.target_position_world_yolo.tx
            self.previous_target_position_y = self.target_position_world_yolo.ty
            self.previous_target_position_z = self.target_position_world_yolo.tz

            self.position_yolo_pub_camera.publish(self.target_position_camera_yolo)
            self.position_yolo_pub.publish(self.target_position_world_yolo)
            self.position_gazebo_pub.publish(self.target_position_world_gazebo)
            self.jackal_pos_gazebo_pub.publish(self.jackal_state_gazebo)
            self.jackal_pos_yolo_pub.publish(self.jackal_state_yolo)


        except Exception as e:
            pass
    



    
if __name__ == '__main__':
    node = PnpNode()
    rospy.spin()
