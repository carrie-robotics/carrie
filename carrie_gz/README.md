# carrie_gz
Contains launch files for launching Carrie robot inside a Gazebo and RViz simulation

## 📂 Directories
- `config` defines a list of topics and message types to be bridged between ROS2 and Gazebo
- `config_gui` defines gazebo scene and plugins that add functionality to the scene
- `launch` python launch files to bring up the simulation
- `rviz` defines rviz settings and widgets
- `urdf` defines the joint states of the robot that are published to ROS2 during Gazebo simulation
- `worlds` different worlds for carrie to be spawned in 


## 🤖 Launch Carrie in Gazebo
### Build the package:
```shell
colcon build --symlink-install
```
### Launch Gazebo (with RViz)
```shell
ros2 launch carrie_gz carrie_gz.launch.py
```
<p align="center"> <img src="docs/carrie_gazebo_empty_world.png" alt="rviz carrie" width="1000"/> </p>

<p align="center"> <img src="docs/carrie_rviz.png" alt="rviz carrie" width="1000"/> </p>

### Launch Gazebo (with Robot State Publisher)
If you are seeing the error in RViz that says "No transform from l_wheel_link to base_link" or "No transform from r_wheel_link to base_link" please launch the RViz with the robot state publisher node
```shell
ros2 launch carrie_gz carrie_gz.launch.py rsp:=True
```

### Launch Gazebo (without RViz and ros bridge)
```shell
ros2 launch carrie_gz carrie_gz.launch.py ros_bridge:=False rviz:=False
```
You can see the available parameters for the launch file:
```shell
ros2 launch carrie_gz carrie_gz.launch.py --show-args
```

### Launching more than one robots 🤖 🤖 🤖
```shell
ros2 launch carrie_gz carrie_gz.launch.py ros_bridge:=False rviz:=False robots:="
    carrie1={x: 0.0, y: 0.0, z: 0.1, yaw: 0.};
    carrie2={x: 1.0, y: 1.0, z: 0.1, yaw: 0.};"
```
<p align="center"> <img src="docs/carrie_robots_gazebo.png" alt="rviz carrie" width="1000"/> </p>