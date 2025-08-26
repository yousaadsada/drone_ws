#!/usr/bin/env python
import rospy
import math
import threading
from gazebo_msgs.msg import LinkStates, LinkState
from gazebo_msgs.srv import SetLinkState
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from my_custom_msgs.msg import DroneState

class BaseLinkPose:
    def __init__(self):
        rospy.init_node('base_link_pose')

        self.running = True
        self.lock = threading.Lock()

        self.current_drone_state = DroneState()
        self.link_state = LinkState()
        self.link_state.reference_frame = "world"

        rospy.wait_for_service('/gazebo/set_link_state')
        self.set_link_pose_srv = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)

        self.sub_quad_msg = rospy.Subscriber(
            '/gazebo/link_states_throttled',
            LinkStates,
            self.subcribe_quadrotor_msg_callback,
            queue_size=1
        )

        self.set_link_pose_thread = threading.Thread(target=self.set_link_pose_thread_callback)
        self.set_link_pose_thread.daemon = True
        self.set_link_pose_thread.start()

    def subcribe_quadrotor_msg_callback(self, msg):
        try:
            index = msg.name.index("quadrotor::link")
        except ValueError:
            return

        with self.lock:
            pose = msg.pose[index]
            twist = msg.twist[index]

            self.current_drone_state.x = pose.position.x
            self.current_drone_state.y = pose.position.y
            self.current_drone_state.z = pose.position.z

            q = [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            ]
            roll, pitch, yaw = euler_from_quaternion(q)
            self.current_drone_state.roll = roll
            self.current_drone_state.pitch = pitch
            self.current_drone_state.yaw = yaw

            self.current_drone_state.x_speed = twist.linear.x
            self.current_drone_state.y_speed = twist.linear.y
            self.current_drone_state.z_speed = twist.linear.z
            self.current_drone_state.roll_speed = twist.angular.x
            self.current_drone_state.pitch_speed = twist.angular.y
            self.current_drone_state.yaw_speed = twist.angular.z

    def set_link_pose_thread_callback(self):
        rate = rospy.Rate(100)  # 100 Hz is safer for Gazebo service calls

        while not rospy.is_shutdown() and self.running:
            with self.lock:
                yaw = self.current_drone_state.yaw
                pitch = self.current_drone_state.pitch

                # Position offset in drone body frame, rotated into world frame
                dx = 0.15 * math.cos(yaw) * math.cos(pitch)
                dy = 0.15 * math.sin(yaw) * math.cos(pitch)
                dz = -0.15 * math.sin(pitch)

                self.link_state.link_name = "frame_link::frame_link"
                self.link_state.pose.position.x = self.current_drone_state.x + dx
                self.link_state.pose.position.y = self.current_drone_state.y + dy
                self.link_state.pose.position.z = self.current_drone_state.z + dz

                quat = quaternion_from_euler(0.0, 0.0, yaw)
                self.link_state.pose.orientation.x = quat[0]
                self.link_state.pose.orientation.y = quat[1]
                self.link_state.pose.orientation.z = quat[2]
                self.link_state.pose.orientation.w = quat[3]

            try:
                self.set_link_pose_srv(self.link_state)
            except rospy.ServiceException as e:
                rospy.logwarn("Failed to set link state: %s" % e)

            rate.sleep()


if __name__ == '__main__':
    try:
        node = BaseLinkPose()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
