cmake_minimum_required(VERSION 3.0.2)
project(my_drone_model)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  std_msgs
  urdf
  xacro
  gazebo_msgs
  std_srvs
)

find_package(gazebo REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
)


catkin_package(
   CATKIN_DEPENDS gazebo_ros roscpp std_msgs urdf xacro gazebo_msgs
)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(wrench_plugin src/wrench_plugin.cpp)

target_link_libraries(wrench_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
)
add_library(attach_detach_plugin SHARED src/attach_detach_plugin.cpp)
target_link_libraries(attach_detach_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


catkin_install_python(PROGRAMS
  scripts/thrust_calculator.py
  scripts/manual_control.py
  scripts/manual_control_jackal.py
  scripts/odometry_plugin.py
  scripts/motorpid_plugin.py
  scripts/quadrotor_motor_controller.py
  scripts/propeller_velocity_publisher.py
  scripts/save_image.py
  scripts/camera_pitch_controller.py
  scripts/camera_pitch_controller_auto.py
  scripts/collect_drone_state.py
  scripts/quadrotor_wrench_control.py
  scripts/move2ref.py
  scripts/frame_link_pose.py
  scripts/collect_keypoint.py
  scripts/draw_cube.py
  scripts/keypoint_detection.py
  scripts/drop_payload_on_jackal_gazebo.py
  scripts/drop_payload_on_jackal_yolo.py
  scripts/drop_payload_on_jackal_yolo_auto.py
  scripts/gripper_controller.py
  scripts/gripper_controller_auto.py
  scripts/delay_spawner.py
  scripts/sequential_launcher.py
  scripts/collect_jackal_figure_drone_trajectory.py
  scripts/get_jackal_pos.py
  scripts/kalman_filter_velocity_estimation.py
  scripts/tracking_trajectory.py
  scripts/jackal_control_auto.py
  scripts/comparison_yolo_gazebo_pos_est.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


