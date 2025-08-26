import rospy
from tf2_msgs.msg import TFMessage
from my_custom_msgs.msg import KeyPoints, JackalSize, DroneState
import math
from geometry_msgs.msg import TransformStamped
import transforms3d
import threading
import numpy as np
from geometry_msgs.msg import Point32
import time
from gazebo_msgs.msg import LinkStates
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_matrix

class CollectKeypoint:
    def __init__(self):
        rospy.init_node('collect_keypoint')
        self.running = True

        self.target_frame_jackal = 'jackal'
        self.target_frame_drone = 'drone'
        self.keypoints_name = [
        "front_top_left",
        "front_top_right",
        "front_bottom_left",
        "front_bottom_right",
        "rear_top_left",
        "rear_top_right",
        "rear_bottom_left",
        "rear_bottom_right"
        ]
        
        self.jackal_size = JackalSize()

        self.jackal_size.length = 0.4
        self.jackal_size.width = 0.5
        self.jackal_size.height = 0.4

        
        self.keypoints_world_frame = KeyPoints()
        self.keypoints_camera_frame = KeyPoints()
        self.keypoints_camera_frame_2d = KeyPoints()

        self.keypoints_jackal_frame = KeyPoints()
        self.keypoints_jackal_frame.front_top_left.x = self.jackal_size.width /2 
        self.keypoints_jackal_frame.front_top_left.y = self.jackal_size.length /2
        self.keypoints_jackal_frame.front_top_left.z = self.jackal_size.height /2

        self.keypoints_jackal_frame.front_top_right.x = self.jackal_size.width /2
        self.keypoints_jackal_frame.front_top_right.y = -self.jackal_size.length /2
        self.keypoints_jackal_frame.front_top_right.z = self.jackal_size.height /2

        self.keypoints_jackal_frame.front_bottom_left.x = self.jackal_size.width /2
        self.keypoints_jackal_frame.front_bottom_left.y = self.jackal_size.length /2
        self.keypoints_jackal_frame.front_bottom_left.z = -self.jackal_size.height /2

        self.keypoints_jackal_frame.front_bottom_right.x = self.jackal_size.width /2
        self.keypoints_jackal_frame.front_bottom_right.y = -self.jackal_size.length /2
        self.keypoints_jackal_frame.front_bottom_right.z = -self.jackal_size.height /2

        self.keypoints_jackal_frame.rear_top_left.x = -self.jackal_size.width /2 
        self.keypoints_jackal_frame.rear_top_left.y = self.jackal_size.length /2
        self.keypoints_jackal_frame.rear_top_left.z = self.jackal_size.height /2

        self.keypoints_jackal_frame.rear_top_right.x = -self.jackal_size.width /2
        self.keypoints_jackal_frame.rear_top_right.y = -self.jackal_size.length /2
        self.keypoints_jackal_frame.rear_top_right.z = self.jackal_size.height /2

        self.keypoints_jackal_frame.rear_bottom_left.x = -self.jackal_size.width /2
        self.keypoints_jackal_frame.rear_bottom_left.y = self.jackal_size.length /2
        self.keypoints_jackal_frame.rear_bottom_left.z = -self.jackal_size.height /2

        self.keypoints_jackal_frame.rear_bottom_right.x = -self.jackal_size.width /2
        self.keypoints_jackal_frame.rear_bottom_right.y = -self.jackal_size.length /2
        self.keypoints_jackal_frame.rear_bottom_right.z = -self.jackal_size.height /2
        

        self.camera_matrix = np.array([
                            [381.04, 0.0, 320.0],
                            [0.0, 381.04, 240.0],
                            [0.0, 0.0, 1.0]
                        ])

        self.camera_state = DroneState()
        self.jackal_state = DroneState()

        self.subcribe_quadrotor_msg = rospy.Subscriber('/gazebo/link_states_throttled', LinkStates, self.subcribe_quadrotor_msg_callback, queue_size=1)
        self.keypoints_camera_frame_2d_publish = rospy.Publisher('/keypoints_2d', KeyPoints, queue_size=1)
        
        self.get_keypoints_thread = threading.Thread(target = self.get_raw_keypoints_thread_callback)
        self.get_keypoints_thread.daemon = True
        self.get_keypoints_thread.start()

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
        

    def get_raw_keypoints_thread_callback(self):
        
        def get_rotation_matrix(roll, pitch, yaw):
            R_x = np.array([[1, 0, 0],
                            [0, np.cos(roll), -np.sin(roll)],
                            [0, np.sin(roll), np.cos(roll)]])

            R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                            [0, 1, 0],
                            [-np.sin(pitch), 0, np.cos(pitch)]])

            R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                            [np.sin(yaw), np.cos(yaw), 0],
                            [0, 0, 1]])
            
            R = np.dot(R_z, np.dot(R_y, R_x))
            return R

        def get_transformation_matrix(x, y, z, R):

            # Create the 4x4 transformation matrix
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = [x, y, z]
            
            return T
        
        def transform_keypoints_world_frame(T):
            def transform_point(T, point):
                    point_homogeneous = np.array([point.x, point.y, point.z, 1.0])
                    point_world_homogeneous = np.dot(T, point_homogeneous)
                    return Point32(x=point_world_homogeneous[0], y=point_world_homogeneous[1], z=point_world_homogeneous[2])

            for i, keypoint_name in enumerate(self.keypoints_name):
                orignal_frame_point = getattr(self.keypoints_jackal_frame, keypoint_name)
                transform_frame_point = transform_point(T, orignal_frame_point)
                setattr(self.keypoints_world_frame, keypoint_name, transform_frame_point)

        def transform_keypoints_camera_frame(T):
            def transform_point(T, point):
                    point_homogeneous = np.array([point.x, point.y, point.z, 1.0])
                    point_world_homogeneous = np.dot(T, point_homogeneous)
                    return Point32(x=point_world_homogeneous[0], y=point_world_homogeneous[1], z=point_world_homogeneous[2])

            for i, keypoint_name in enumerate(self.keypoints_name):
                orignal_frame_point = getattr(self.keypoints_world_frame, keypoint_name)
                transform_frame_point = transform_point(T, orignal_frame_point)
                setattr(self.keypoints_camera_frame, keypoint_name, transform_frame_point)

        while self.running:
            camera_roll = self.camera_state.roll
            camera_pitch = self.camera_state.pitch
            camera_yaw = self.camera_state.yaw
            camera_x = self.camera_state.x
            camera_y = self.camera_state.y
            camera_z = self.camera_state.z

            jackal_roll = self.jackal_state.roll
            jackal_pitch = self.jackal_state.pitch
            jackal_yaw = self.jackal_state.yaw
            jackal_x = self.jackal_state.x
            jackal_y = self.jackal_state.y
            jackal_z = self.jackal_state.z


            R_jackal2world = get_rotation_matrix(jackal_roll, jackal_pitch, jackal_yaw)
            T_jackal2world = get_transformation_matrix(jackal_x, jackal_y, jackal_z, R_jackal2world)
            #from self.keypoints_parrotframe to self.keypoints_world_frame
            transform_keypoints_world_frame(T_jackal2world)

            R_camera2world = get_rotation_matrix(camera_roll, camera_pitch, camera_yaw)
            T_camera2world = get_transformation_matrix(camera_x, camera_y, camera_z, R_camera2world)
            T_world2camera = np.linalg.inv(T_camera2world)
            #from self.keypoints_world_frame to self.keypoints_anafi_frame
            transform_keypoints_camera_frame(T_world2camera)

            for i, keypoint_name in enumerate(self.keypoints_name):
                keypoint3d_receive = getattr(self.keypoints_camera_frame, keypoint_name)
                keypoint_3d = np.zeros(3)
                keypoint_3d[0] = -keypoint3d_receive.y
                keypoint_3d[1] = -keypoint3d_receive.z
                keypoint_3d[2] = keypoint3d_receive.x

                keypoint_2d_homogeneous = np.dot(self.camera_matrix, keypoint_3d)
                if keypoint_2d_homogeneous[2] != 0:
                    keypoint_2d = keypoint_2d_homogeneous[:2] / keypoint_2d_homogeneous[2]
                else:
                    keypoint_2d = np.array([float('inf'), float('inf')])

                keypoint_2d_point32 = Point32(x=keypoint_2d[0], y=keypoint_2d[1], z=0.0)
                setattr(self.keypoints_camera_frame_2d, keypoint_name, keypoint_2d_point32)
        
            self.keypoints_camera_frame_2d_publish.publish(self.keypoints_camera_frame_2d)

            time.sleep(0.01)




if __name__ == '__main__':
    node = CollectKeypoint()
    rospy.spin()
