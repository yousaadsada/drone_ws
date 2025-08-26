# import rospy
# from sensor_msgs.msg import JointState
# from std_msgs.msg import Float64
# from geometry_msgs.msg import Wrench

# class ThrustCalculator:
#     def __init__(self):
#         rospy.init_node('thrust_calculator', anonymous=True)
#         self.kt = 0.00000245  # Thrust coefficient

#         # Publishers for each rotor's force
#         self.force_pub1 = rospy.Publisher('/quadrotor/rotor1_force', Wrench, queue_size=10)
#         self.force_pub2 = rospy.Publisher('/quadrotor/rotor2_force', Wrench, queue_size=10)
#         self.force_pub3 = rospy.Publisher('/quadrotor/rotor3_force', Wrench, queue_size=10)
#         self.force_pub4 = rospy.Publisher('/quadrotor/rotor4_force', Wrench, queue_size=10)

#         # Subscriber to joint states
#         rospy.Subscriber('/quadrotor/joint_states', JointState, self.joint_state_callback)

#     def joint_state_callback(self, msg):
#         # Extract velocities for each rotor
#         velocities = msg.velocity
#         names = msg.name

#         # Calculate and publish force for each rotor
#         for i, name in enumerate(names):
#             omega = velocities[i]
#             force = self.kt * (omega ** 2)
#             force_msg = Wrench()
#             force_msg.force.z = force  # Set the force in Newtons (e.g., from K_t * omega^2)
#             force_msg.force.x = 0.0
#             force_msg.force.y = 0.0
#             force_msg.torque.x = 0.0
#             force_msg.torque.y = 0.0
#             force_msg.torque.z = 0.0
#             if name == "rotor1_joint":
#                 self.force_pub1.publish(force_msg)
#             elif name == "rotor2_joint":
#                 self.force_pub2.publish(force_msg)
#             elif name == "rotor3_joint":
#                 self.force_pub3.publish(force_msg)
#             elif name == "rotor4_joint":
#                 self.force_pub4.publish(force_msg)

#     def run(self):
#         rospy.spin()

# if __name__ == '__main__':
#     try:
#         node = ThrustCalculator()
#         node.run()
#     except rospy.ROSInterruptException:
#         pass


#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

class ThrustCalculator:
    def __init__(self):
        rospy.init_node('thrust_calculator', anonymous=True)

        # Publishers for rotor velocity commands
        self.vel_pub1 = rospy.Publisher('/quadrotor/rotor1_velocity_controller/command', Float64, queue_size=10)
        self.vel_pub2 = rospy.Publisher('/quadrotor/rotor2_velocity_controller/command', Float64, queue_size=10)
        self.vel_pub3 = rospy.Publisher('/quadrotor/rotor3_velocity_controller/command', Float64, queue_size=10)
        self.vel_pub4 = rospy.Publisher('/quadrotor/rotor4_velocity_controller/command', Float64, queue_size=10)

        # Desired velocity for hover (adjust as needed)
        self.velocity = 1000.51  # rad/s for 1 kg drone hover
        # For lift-off, use 1100.0 rad/s
        # self.velocity = 1100.0

    def run(self):
        rate = rospy.Rate(50)  # 50 Hz, matching joint_state_controller
        rospy.loginfo("Publishing rotor velocity commands...")
        while not rospy.is_shutdown():
            # Publish velocities, accounting for rotor directions (CCW/CW)
            self.vel_pub1.publish(self.velocity)   # rotor1: CCW, positive
            self.vel_pub2.publish(-self.velocity)  # rotor2: CW, negative
            self.vel_pub3.publish(self.velocity)   # rotor3: CCW, positive
            self.vel_pub4.publish(-self.velocity)  # rotor4: CW, negative
            rate.sleep()

if __name__ == '__main__':
    try:
        node = ThrustCalculator()
        node.run()
    except rospy.ROSInterruptException:
        pass