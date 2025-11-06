# <img src="docs/carrie-icon1-512x512.png" width="24" height="24" alt="icon" /> carrie
Building a mobile manipulator based on a [Fetch Mobile Manipulator](https://github.com/ZebraDevs/fetch_ros) chassis.
<p align="center">
  <img src="docs/fetch13_chassis1.jpeg" alt="fetch 13 chassis" width="400" />
</p>

# Setup

Install [pixi](https://pixi.sh) if you haven't already:
```shell
curl -fsSL https://pixi.sh/install.sh | bash
```

Install dependencies:
```shell
pixi install
```

Build the repository:
```shell
pixi run build
```

## Test

To test that your environment is working with graphics:
```shell
pixi run view-carrie
```

# Available Pixi Tasks

```shell
pixi run build         # Build the workspace with colcon
pixi run clean         # Clean build artifacts
pixi run test          # Run colcon tests
pixi run view-carrie   # Launch RViz to view Carrie
pixi run sim-headless  # Launch Gazebo simulation (headless)
pixi run sim-gui       # Launch Gazebo simulation with GUI and RViz
pixi run sim-rsp       # Launch Gazebo with Robot State Publisher and RViz
pixi run sim-multi     # Launch Gazebo with multiple robots (GUI enabled)
```

## Custom Commands

You can run any ROS 2 command with pixi by sourcing the install directory:

```shell
# Run a custom launch with arguments
pixi run bash -c 'source install/setup.bash && ros2 launch carrie_gz carrie_gz.launch.py <your-args>'

# Or enter a pixi shell and work interactively
pixi shell
source install/setup.bash
ros2 launch carrie_gz carrie_gz.launch.py robots="carrie1={x: 0.0, y: 0.0, z: 0.1, yaw: 0.}; carrie2={x: 2.0, y: 0.0, z: 0.1, yaw: 1.57};"
```
