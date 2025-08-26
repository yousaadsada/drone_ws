# Anafi Drone Control System

[output.webm](https://github.com/user-attachments/assets/6eb0b561-75a4-4ca8-ac12-42ab158c41ff)

This project employs Gazebo to simulate precise payload drops from a quadrotor, modeled with six degrees of freedom. Model Predictive Control (MPC) is used to guide the quadrotor along a desired trajectory. A camera mounted on the quadrotor captures images of a moving vehicle, and a YOLO-based eight-keypoint model is trained to estimate the vehicle's 3D position. A Kalman filter is implemented to refine the vehicle's position and velocity estimates. The quadrotor successfully tracks and delivers the payload to the moving vehicle using visual detection.

## Table of Contents

- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Contact](#contact)

---

## Features

✅ Tracking trajectory through MPC

✅ Pose and velocity estimation by visual detection.

✅ Deliver payload on moving vehicle.


---

## Installation

### Prerequisites

- Ubuntu 20.04  
- ROS Noetic  
- Python 3.8+
- CUDA 11.8 or above (for GPU acceleration)   
- OpenCV, NumPy, SciPy, CasADi, cv_bridge, sensor_msgs, geometry_msgs, etc.

### Clone Repository

```bash
git clone https://github.com/zjiang11/anafi_ws.git
```


## Usage

### Build Program

```bash
cd ~/anafi_ws
colcon build
source install/setup.bash
```

### Derive the Quaternions for Representing Pose
The following code use /tf topic with high frequency (above 100Hz) to subscribe ros2 message of 'TFMessage' for the pursuing drone (Anafi) and target drone (Bebop1).
One way to accquire the TFMessage for both drones is to use Vicon System in Martin Barczyk's lab, (211, Mechenical Engineering Building, University of Alberta). The detail of setting up the Vicon system can be refered to https://gitlab.com/barczyk-mechatronic-systems-lab/anafi_ros2/-/tree/joshua?ref_type=heads

### Manual Control

```bash
ros2 run anafi_test manual_control.py
```
Notice: 

key.right: Take off

key.left: Landing

w: X Direction (Forward)

s: X Direction (Backward)

a: Y Direction (Left)

d: Y Direction (Right)

r: Z Direction (Upward)

f: Z Direction (Downward)

c: Yaw Direction (Anticlockwsie)

x: Yaw Direction (Clockwise)


### Test Anafi Drone System state

#### Linear MPC

Implementing linear state function for MPC on drone

##### Collect Anafi Drone's Data

```bash
ros2 run anafi_test collect_anafi_data_linear_mpc.py
```
notice:

Enter testing axis after using "key.right" to take off the drone.

Using keyboard for manual control (right, left, w, s, a, d, r, f, c, x) can swith from the tracking mode to manual mode for safety.

##### Calculate and test Anafi Drone's System State Matrix

```bash
python3 src/anafi_test/process_data/calculate_state_function_linear_mpc.py
python3 src/anafi_test/process_data/test_state_function_linear_mpc.py
```

#### Newton-Euler MPC

Implementing reference roll, pitch angles for MPC

##### Collect Anafi Drone's Data

```bash
ros2 run anafi_test collect_anafi_data_newton_euler_mpc.py
```
notice:

Enter testing axis after using "key.right" to take off the drone.

Using keyboard for manual control (right, left, w, s, a, d, r, f, c, x) can swith from the tracking mode to manual mode for safety.

##### Calculate and test Anafi Drone's System State Matrix

```bash
python3 src/anafi_test/process_data/calculate_state_function_newton_euler_mpc.py
python3 src/anafi_test/process_data/test_state_function_newton_euler_mpc.py
```

### Move to Reference Point by Linear MPC

```bash
ros2 run anafi_test move2point_linear_mpc.py
```

notice:

input four float value: x,y,z,yaw for reference point

Using keyboard for manual control (right, left, w, s, a, d, r, f, c, x) can swith from the MPC mode to manual mode for safety.

### Track the Reference Trajectory by Newton-Euler MPC

```bash
ros2 run anafi_test move2point_linear_mpc.py
```

notice:

input eight float value: x,y,z,yaw for initial reference point. x_speed, y_speed, z_speed, yaw_speed for reference point moving speed.

The trajectory is in the shape of the circle. Larger x_speed, y_speed will enhance radius of the circle. The x_speed, y_speed, z_speed and yaw_speed should not exceed 0.3 at the first attempt. User can adjust the trajectory by revising "def update_ref_state_callback" in move2point_linear_mpc.py

Using keyboard for manual control (right, left, w, s, a, d, r, f, c, x) can swith from the MPC mode to manual mode for safety.

### Plot the Drone's Trajectory

```bash
python3 src/anafi_test/process_data/plot_move2point_linear_mpc.py
```

![截图 2025-04-13 01-30-42](https://github.com/user-attachments/assets/dcb33b75-d015-4456-92e1-5818815513a1)

```bash
python3 src/anafi_test/process_data/plot_move2point_newton_euler_mpc.py
```

![截图 2025-04-13 01-31-59](https://github.com/user-attachments/assets/4f25880c-a23c-4ce9-a39c-221b35ad4e0c)

![截图 2025-04-13 01-32-38](https://github.com/user-attachments/assets/c9bbc890-0368-4e73-b9ac-aac83d7f02eb)

### Collect Parrot Drone's Figures

```bash
ros2 run track_parrot collect_parrot_fig.py
```
### Label 2D Images for YOLO 2D BBox Model Training

```bash
pip install labelImg
labelImg
```

### Collect Parrot Drone's Keypoints for YOLO 3D BBox Model Training

```bash
ros2 launch get_keypoint_launch.py
```

### Train and Test YOLO 2D BBox

```bash
python3 src/track_parrot/train_drone_yolo_2d/train.py
python3 src/track_parrot/train_drone_yolo_2d/test.py
```
![截图 2025-04-13 01-27-50](https://github.com/user-attachments/assets/56b1ee1e-487c-49c7-b488-d9c8e5a932ff)


### Train YOLO 3D BBox

```bash
python3 src/track_parrot/train_drone_yolo_3d/train.py
python3 src/track_parrot/train_drone_yolo_3d/test.py
```
![截图 2025-04-13 01-27-10](https://github.com/user-attachments/assets/ba60f899-9bec-4ac3-b215-59e9054e75d7)


### Persue the Target Drone

```bash
ros2 launch pursuer_launch_yolo.py
```
notice:

Click 'Start Tracking' after using "key.right" to take off the drone.

Using keyboard for manual control (right, left, w, s, a, d, r, f, c, x) can swith from the tracking mode to manual mode for safety.

### PLot Tracking Process

```bash
python3 src/track_parrot/process_data/plot_tracking_data.py
```

![截图 2025-04-13 01-28-55](https://github.com/user-attachments/assets/d4c14230-8e24-4abe-8dba-c600f4695c7d)


## Contact
Email: zjiang11@ualberta.ca
