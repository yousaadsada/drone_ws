#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import LinkStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist

class LinkStatesToOdometry:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('link_states_to_odometry', anonymous=True)

        # Publisher for odometry messages
        self.odom_pub = rospy.Publisher('/odometry', Odometry, queue_size=10)

        # Subscriber to /gazebo/link_states
        self.link_states_sub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.link_states_callback)

        # Link name to track
        self.target_link = 'quadrotor::frame'  # Matches your /gazebo/link_states output

        # Odometry message
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = 'map'  # Parent frame (consistent with your previous config)
        self.odom_msg.child_frame_id = 'frame'  # Child frame (link name)

        rospy.loginfo("Initialized link_states_to_odometry node, tracking link: %s", self.target_link)

    def link_states_callback(self, msg):
        try:
            # Find the index of the target link
            if self.target_link in msg.name:
                index = msg.name.index(self.target_link)
                
                # Populate odometry message
                self.odom_msg.header.stamp = rospy.Time.now()
                self.odom_msg.pose.pose = msg.pose[index]
                self.odom_msg.twist.twist = msg.twist[index]

                # Publish odometry message
                self.odom_pub.publish(self.odom_msg)
            else:
                rospy.logwarn_once("Target link %s not found in /gazebo/link_states", self.target_link)
        except Exception as e:
            rospy.logerr("Error processing link_states: %s", str(e))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        converter = LinkStatesToOdometry()
        converter.run()
    except rospy.ROSInterruptException:
        pass