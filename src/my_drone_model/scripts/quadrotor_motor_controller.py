
#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray, Float64

class QuadrotorMotorController:
    def __init__(self):
        rospy.init_node('quadrotor_motor_controller', anonymous=True)
        self.rate = rospy.Rate(100)
        self.model_name = rospy.get_param('~model_name', 'quadrotor')
        self.vel_cmd_topic = f"/{self.model_name}/vel_cmd"
        self.max_speed = 104.72  # Max 1000 RPM (rad/s)
        self.command_received = False  # Track if valid command received

        self.desired_speeds = [0.0] * 4  # Explicitly initialize to zero

        self.motor_pubs = [
            rospy.Publisher(f'/{self.model_name}/rotor{i+1}_controller/command', Float64, queue_size=10)
            for i in range(4)
        ]
        rospy.Subscriber(self.vel_cmd_topic, Float64MultiArray, self.vel_cmd_callback)

        # Publish zero commands initially to ensure no motion
        for pub in self.motor_pubs:
            pub.publish(Float64(0.0))

        rospy.loginfo(f"[{self.model_name}] Motor controller initialized. Waiting for velocity commands on {self.vel_cmd_topic}...")

    def vel_cmd_callback(self, msg):
        self.command_received = True
        for i in range(min(len(msg.data), 4)):
            self.desired_speeds[i] = max(min(msg.data[i], self.max_speed), -self.max_speed)
        rospy.loginfo(f"[{self.model_name}] Received desired speeds: {self.desired_speeds}")

    def control_loop(self):
        while not rospy.is_shutdown():
            if not self.command_received:
                self.desired_speeds = [0.0] * 4  # Ensure zero until command received
                rospy.loginfo_throttle(1.0, f"[{self.model_name}] No command received, publishing zero speeds")
            for i in range(4):
                self.motor_pubs[i].publish(Float64(self.desired_speeds[i]))
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = QuadrotorMotorController()
        controller.control_loop()
    except rospy.ROSInterruptException:
        pass
