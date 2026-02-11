# carrie_perception
Contains a ros2 node for performing prompt based image segmentation using SAM3 foundation model. This node subscribe to the camera topic and publishes the 2D detections and the binary segmentation masks based on the text prompt. 

## 📂 Directories
- `carrie_perception` contains the sam3 node that performs the segmentation. The entire node runs irrespective of CUDA. It runs on mps (on macos), if mps is not detected then the node run on the cpu.   
- `config` parameters that effect the segmentation can be set here 
- `example_images` contains some sample images on which segmentation could be run and output could be visualized 

## 🤖 Launch Carrie in Gazebo
### Build the package:
```shell
colcon build --symlink-install
```
### Launch Gazebo (without gazebo gui and RViz)
The default behavior of the launch file is to start the Gazebo server without its GUI and without launching RViz. 
```shell
ros2 launch carrie_gz carrie_gz.launch.py
```
You will see the terminal log confirming the activation of the gazebo server. 

### Launch Gazebo (with gazebo gui and RViz)
```shell
ros2 launch carrie_gz carrie_gz.launch.py gazebo_headless:=False rviz:=True
```
<p align="center"> <img src="docs/carrie_gazebo_empty_world.png" alt="rviz carrie" width="1000"/> </p>

<p align="center"> <img src="docs/carrie_rviz.png" alt="rviz carrie" width="1000"/> </p>

### Launch Gazebo (with Robot State Publisher)
If you are seeing the error in RViz that says "No transform from l_wheel_link to base_link" or "No transform from r_wheel_link to base_link" please launch the RViz with the robot state publisher node
```shell
ros2 launch carrie_gz carrie_gz.launch.py rsp:=True rviz:=True
```
<p align="center"> <img src="docs/carrie_rviz_rsp.png" alt="rviz carrie" width="1000"/> </p>

### Launching more than one robots 🤖 🤖 🤖
```shell
ros2 launch carrie_gz carrie_gz.launch.py ros_bridge:=False gazebo_headless:=False robots:="
    carrie1={x: 0.0, y: 0.0, z: 0.1, yaw: 0.};
    carrie2={x: 1.0, y: 1.0, z: 0.1, yaw: 0.};"
```
<p align="center"> <img src="docs/carrie_robots_gazebo.png" alt="rviz carrie" width="1000"/> </p>