# ROS2 UR3e Robotiq Workspace

This project contains a ROS2 workspace with UR3e robot and Robotiq 2F-85 gripper description, plus a Dockerfile to run the environment.

## Contents

- ROS2 workspace source files (in ros2_ws_src)
- Dockerfile to build ROS2 + UR3e + Robotiq container
- entrypoint.sh script for container startup
- bashrc for custom shell environment

## How to build Docker image

```bash
docker build -t alielgendy50/ros2_ur3e_gripper_robotiq_2f_85:latest .
```

## How to run container

```bash
docker run -it --rm   --name ros2_ur3e_gripper_robotiq_2f_85   alielgendy50/ros2_ur3e_gripper_robotiq_2f_85:latest
```

## Launching the URDF model

Inside the container:

```bash
source /ros2_ws/install/setup.bash
ros2 launch urdf_tutorial display.launch.py model:=/ros2_ws/src/ur_robotiq/ur_robotiq_description/urdf/ur3e_urdf.xacro
```

## Notes

- This repo does not contain build, install, or log folders.
- Make sure to build the Docker image before running.
