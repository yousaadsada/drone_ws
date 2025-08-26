#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

def main():
    rospy.init_node('propeller_velocity_publisher')

    pub1 = rospy.Publisher('/propeller_1_joint_velocity_controller/command', Float64, queue_size=1)
    pub2 = rospy.Publisher('/propeller_2_joint_velocity_controller/command', Float64, queue_size=1)
    pub3 = rospy.Publisher('/propeller_3_joint_velocity_controller/command', Float64, queue_size=1)
    pub4 = rospy.Publisher('/propeller_4_joint_velocity_controller/command', Float64, queue_size=1)

    rate = rospy.Rate(100)  # 1 Hz

    while not rospy.is_shutdown():
        msg = Float64()
        msg.data = 100.0
        pub1.publish(msg)
        pub2.publish(-msg)
        pub3.publish(msg)
        pub4.publish(-msg)
        rate.sleep()

if __name__ == '__main__':
    main()
