# 🤖 ROS2 UR3e Robotiq Workspace

This project provides a complete ROS2 workspace and Docker environment for simulating the **Universal Robots UR3e** robotic arm with the **Robotiq 2F-85** gripper. It’s designed for easy setup, repeatability, and direct use in development or testing environments.

---

## 📁 Project Structure

```
ros2-ur3e-robotiq/
├── Dockerfile             # Builds ROS2 + UR3e + Robotiq image
├── entrypoint.sh          # Startup script for the container
├── bashrc                 # Custom shell setup for container
├── ros2_ws_src/           # ROS2 workspace source files
│   └── ur_robotiq/
│       ├── ur_robotiq/                # ROS2 package
│       │   ├── CMakeLists.txt
│       │   ├── LICENSE
│       │   └── package.xml
│       └── ur_robotiq_description/    # Description package
│           ├── CMakeLists.txt
│           ├── LICENSE
│           ├── package.xml
│           ├── urdf/
│           │   ├── robotiq_2f_85_urdf.xacro
│           │   ├── ur3e_robotiq_2f_85_urdf.xacro
│           │   └── ur3e_urdf.xacro
│           ├── include/
│           └── src/
```

---

## 🐳 Docker Environment

### 🔨 Build the Docker Image

```bash
docker build -t alielgendy50/ros2_ur3e_gripper_robotiq_2f_85:latest .
```

> 🧠 Note: This image is based on a ROS2 environment and includes all required dependencies.

### ⬇️ Pull the Image from Docker Hub (Optional)

If you prefer not to build it yourself:

```bash
docker pull alielgendy50/ros2_ur3e_gripper_robotiq_2f_85:latest
```

---

## ▶️ Running the Container

Run the Docker container interactively:

```bash
docker run -it --rm \
  --name ros2_ur3e_gripper_robotiq_2f_85 \
  alielgendy50/ros2_ur3e_gripper_robotiq_2f_85:latest
```

---

## 🚀 Launching the URDF Model

Once inside the running container:

```bash
source /ros2_ws/install/setup.bash

ros2 launch urdf_tutorial display.launch.py \
  model:=/ros2_ws/src/ur_robotiq/ur_robotiq_description/urdf/ur3e_urdf.xacro
```

> If `/ros2_ws/install` does not exist (e.g., first run), build the workspace first:
>
> ```bash
> cd /ros2_ws
> colcon build
> source install/setup.bash
> ```

---

## 📝 Notes

- This repository **excludes** the following folders for clarity and size:
  - `build/`
  - `install/`
  - `log/`
- These are generated at runtime or during build and should not be tracked.
- Make sure your system supports Docker with GUI if you plan to visualize the URDF model (e.g., RViz).

---

## 📦 Credits

- Universal Robots UR3e Description: [UniversalRobots](https://github.com/UniversalRobots)
- Robotiq 2F-85 Gripper Description: [Robotiq](https://github.com/ros-industrial/robotiq)

---

## 🔗 Author

**Ali Wael Elgendy**  
🔗 [LinkedIn](https://www.linkedin.com/in/ali-elgendy-18913b1aa)  
🐙 GitHub: [alielgendy50](https://github.com/alielgendy50)
