# Autonomous Mobile Robot (AMR) Project

This project features a Differential Drive robot using ROS 2 for localization, mapping (SLAM), and autonomous path planning (Navigation).

📍 **Repository:** [https://github.com/aykayy1/mobile_robot_amr](https://github.com/aykayy1/mobile_robot_amr)

## 📂 Project Structure
The project is organized into the following functional modules:
* `/slam`: Contains configuration and launch files for Mapping (SLAM).
* `/navigation`: Contains Nav2 configurations and launch files for autonomous navigation.
* `/agv0509test6`: *(Internal testing directory - Please ignore)*.

## 🛠️ Installation

### 1. System Requirements
* Ubuntu 22.04 LTS
* ROS 2 Humble Hawksbill

### 2. Download and Build
# 1. Create a workspace
```bash
mkdir -p ~/amr_ws/src
cd ~/amr_ws/src
```
# 2. Clone the repository
```bash
git clone [https://github.com/aykayy1/mobile_robot_amr.git](https://github.com/aykayy1/mobile_robot_amr.git)
```
# 3. Install dependencies
```bash
cd ~/amr_ws
rosdep install --from-paths src --ignore-src -r -y
```
# 4. Build the package
```bash
colcon build --symlink-install
source install/setup.bash
```

🚀 Usage
To operate the robot, please open separate terminals for each of the following tasks.

📍 Phase 1: Mapping (SLAM)
Use the modules in the slam directory to scan and build an environment map.

Start SLAM:
(This command initializes the Lidar and the SLAM Toolbox algorithm).
```bash
# Execute the launch file in the slam directory (change the .py filename if necessary)
ros2 launch amr_slam agv_runall.py
```
(This command runs the keyboard node to control the robot manually).
```bash
ros2 run amr_slam wheel_vel_node
```
Save Map (After scanning is complete):
```bash
ros2 run nav2_map_server map_saver_cli -f ~/amr_ws/src/mobile_robot_amr/navigation/maps/my_map
```

📍 Phase 2: Navigation
Use the modules in the navigation directory to enable the robot to navigate autonomously on the saved map.

Start Navigation:
The system will load "my_map" and initialize Nav2.
```bash
# Execute the launch files in the navigation directory
ros2 launch amr_navigation bringup_localization.launch.py
ros2 launch amr_navigation navigation_real_launch.py
ros2 run amr_navigation wheel_vel_node_nav
```
Control:

Use the 2D Pose Estimate tool in RViz to set the robot's initial position.

Use Nav2 Goal to select a destination.

⚠️ Notes
This project focuses on the research and implementation of autonomous mobile robots (AMR) utilizing the ROS 2 framework.

The agv0509test6 directory contains legacy test files and is not part of the standard operating procedure.

Ensure you have run source install/setup.bash in every new terminal window.

👥 Contact
Authors: 
Trần Anh Khoa - [trankhoavt85@gmail.com]
Lê Đức Mạnh   - [manh1472003@gmail.com]

VIDEO DEMO 1:
<div align="center">
  <a href="https://www.youtube.com/watch?v=YFsDgfYz9JY">
    <img src="http://img.youtube.com/vi/YFsDgfYz9JY/0.jpg" width="500" alt="Demo 1 _ ROS 2-Based Autonomous Mobile Robots (AMRs)">
  </a>
</div>

VIDEO DEMO 2:
<div align="center">
  <a href="https://www.youtube.com/watch?v=3QMen4Jf0rc">
    <img src="http://img.youtube.com/vi/3QMen4Jf0rc/0.jpg" width="500" alt="Demo 2 _ ROS 2-Based Autonomous Mobile Robots (AMRs)">
  </a>
</div>

VIDEO PROJECT:
<div align="center">
  <a href="https://www.youtube.com/watch?v=5v9YPMIHcok">
    <img src="https://img.youtube.com/vi/5v9YPMIHcok/0.jpg" width="500" alt="Project Graduation _ ROS 2-Based Autonomous Mobile Robots (AMRs)">
  </a>
</div>
