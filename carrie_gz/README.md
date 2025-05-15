# carrie_gz
Contains launch files for launching Carrie robot inside a Gazebo and RViz simulation

## 📂 Directories
- `config` defines a list of topics and message types to be bridged between ROS2 and Gazebo
- `config_gui` defines gazebo scene and plugins that add functionality to the scene
- `launch` python launch files to bring up the simulation
- `rviz` defines rviz settings and widgets
- `urdf` defines the joint states that are published to ROS 2 during Gazebo simulation
- `worlds` different worlds for carrie to be spawned in 


## 🥃🧊🍒 View Carrie in Gazebo

Setup:
```shell
sudo apt update
```
```shell
sudo apt upgrade
```
```shell
rosdep update
```
```shell
rosdep install --from-paths src -y --ignore-src
```

Build the package
```shell
colcon build --symlink-install
```
Launch Gazebo
```shell
ros2 launch carrie_gz carrie_gz.launch.py
```

If you want to launch Carrie into an empty world
```shell
ros2 launch carrie_gz carrie_gz.launch.py world_name:=empty.sdf
```

<p align="center"> <img src="docs/carrie_gazebo.png" alt="rviz carrie" width="1000"/> </p>

<p align="center"> <img src="docs/carrie_rviz.png" alt="rviz carrie" width="1000"/> </p>