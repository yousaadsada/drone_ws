import rospy
import subprocess
import time

def run(cmd, wait=True):
    rospy.loginfo("Running: " + cmd)
    p = subprocess.Popen(cmd, shell=True)
    if wait:
        p.wait()
    return p

if __name__ == '__main__':
    rospy.init_node('sequential_launcher', anonymous=True)

    # 1. Load quadrotor and ball URDFs
    run("rosparam load $(rospack find my_drone_model)/urdf/quadrotor.urdf quadrotor/robot_description")
    run("rosparam load $(rospack find my_drone_model)/urdf/ball.urdf ball/robot_description")

    # 2. Spawn quadrotor
    run("rosrun gazebo_ros spawn_model -param quadrotor/robot_description -urdf -model quadrotor -z 0.075")

    # 3. Wait for 10 seconds
    rospy.loginfo("Sleeping for 10 seconds...")
    time.sleep(10)

    # 4. Spawn ball
    run("rosrun gazebo_ros spawn_model -param ball/robot_description -urdf -model ball -z 0.01")

    # 5. Start remaining nodes (one-by-one or in groups)
    run("rosrun my_drone_model/gripper_controller.py", wait=False)
    run("rosrun my_drone_model/camera_pitch_controller.py", wait=False)
    run("rosrun my_drone_model/manual_control_jackal.py", wait=False)
    run("rosrun my_drone_model/drop_payload_on_jackal_gazebo.py", wait=False)
    run("rosrun my_drone_model/quadrotor_wrench_control.py", wait=False)

    # 6. Spawn Jackal
    run("roslaunch jackal_gazebo spawn_jackal.launch x:=2 y:=0 z:=1.0 yaw:=0")

    # 7. Load controllers
    run("rosparam load $(rospack find my_drone_model)/config/propeller_controller.yaml")
    run("rosrun controller_manager spawner "
        "quadrotor/quadrotor_propeller_1_joint_velocity_controller "
        "quadrotor/quadrotor_propeller_2_joint_velocity_controller "
        "quadrotor/quadrotor_propeller_3_joint_velocity_controller "
        "quadrotor/quadrotor_propeller_4_joint_velocity_controller "
        "quadrotor/quadrotor_camera_pitch_controller "
        "quadrotor/left_finger_controller "
        "quadrotor/right_finger_controller")

    rospy.loginfo("All nodes launched in order.")