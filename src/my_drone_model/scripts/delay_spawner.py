import rospy
import time
import os

if __name__ == '__main__':
    rospy.init_node('delayed_spawner')

    rospy.loginfo("Spawning quadrotor...")
    os.system("rosrun gazebo_ros spawn_model -param quadrotor/robot_description -urdf -model quadrotor -z 0.075")

    rospy.loginfo("Waiting 10 seconds...")
    time.sleep(10)

    rospy.loginfo("Spawning ball...")
    os.system("rosrun gazebo_ros spawn_model -param ball/robot_description -urdf -model ball -z 0.01")

    rospy.loginfo("Done.")
