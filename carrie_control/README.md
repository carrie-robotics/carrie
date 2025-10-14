# carrie_control
To control Carrie and interface it with ROS2, we use ROS2 Control. ROS2 Control manages the communication between controllers and the robot’s hardware or simulated hardware in Gazebo. 

## How the Control System Was Set Up (ROS2 Control)
### Step 1: Define the urdf
- ` 📂 urdf directory:` This is the folder where the control code for our robot goes. Carrie is currently defined by two urdf files. The first urdf file resides inside the `carrie_description` folder which defines the joints and links of carrie. This second urdf file defines the control stuff for our robot. These two urdf files are together fed into `/robot_description` topic. The most important information here is that we need to fill the `<ros2_control>` and `<gazebo>` tags in this urdf file.  

    `<ros2_control>` → defines what the control system looks like (hardware + controllers).

    `<gazebo>` → loads a plugin so that Gazebo can simulate that hardware interface.

### Step 2: Define the parameters

- `📂 config directory:` We need parameters for the controller manager. These parameters are defined inside the `carrie_controllers.yaml` file. Here we also give our controllers a name and choose the type of the controllers we want to use inoder to control our robot. We have used two controllers: 

    `differential_drive_controller` → a controller for differential drive wheel systems
    `joint_state_broadcaster` → a non-controlling controller to make sure that the wheel transforms are generated correctly 


### Step 3: Launch everthing!
- `📂 launch directory:` finally in the launch folder we launch the `control_manager` and the two controllers i.e., the `differential_drive_controller` and the `joint_state_broadcaster` .

## 🤖 Launch Carrie in Gazebo
### Build the package:
```shell
colcon build --symlink-install
```
### Launch Gazebo (with gazebo gui and RViz) 
```shell
ros2 launch carrie_gz carrie_gz.launch.py rviz:=True gazebo_headless:=False
```
You will see the terminal log confirming the activation of the gazebo server. 

**Note:** Ignore the error on RViz in the `Robot Model` widget. It will go away once you launch the controllers (next step!). 

### Launch the Control launch file 
```shell
ros2 launch carrie_control carrie_control.launch.py
```

you will see the logs confirming the activation of the two controllers

<p align="center"> <img src="docs/ros2_control feedback.png" alt="rviz carrie" width="1000"/> </p>


### Publish the velocity commands!

Before running this command on a new terminal change the `Fixed Frame` widget in RViz to `odom`

```shell
ros2 topic pub /diff_controller/cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}}"
```