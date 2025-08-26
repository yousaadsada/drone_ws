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
git clone https://github.com/yousaadsada/drone_ws.git
```

## Usage

### Build Program

```bash
cd ~/drone_ws
catkin_make
source devel/setup.bash
```


### Tracking trajectory through MPC

```bash
roslaunch my_drone_model tracking_trajectory.launch
```

<img width="1000" height="1200" alt="drone_trajectory_2 0_2 0" src="https://github.com/user-attachments/assets/1d5d5979-74af-4079-bce4-51a7e3446285" />


### Pose and velocity estimation by visual detection.


```bash
roslaunch my_drone_model comparison_yolo_gazebo.launch
```

<img width="1000" height="1000" alt="image" src="https://github.com/user-attachments/assets/8e633a12-8e01-416b-b4c3-6bf61c34ce5c" />

<img width="1000" height="1000" alt="image" src="https://github.com/user-attachments/assets/72d85295-8f63-4473-b171-ba8b25bc192f" />




### Deliver payload on moving vehicle

```bash
roslaunch my_drone_model drop_payload_on_jackal_yolo_auto.launch
```
The gazebo simulation result is shown in the video above

```
```

## Contact
Email: zjiang11@ualberta.ca
