# carrie_description
Contains carrie's kinematic configuration, collision, and visualization meshes.

## 📂 Directories
- `launch` files for robot bringup and visualization
- `meshes` collision and visualization meshes
- `rviz` reusable configurations for development
- `urdf` kinematic configuration of carrie

`carrie_description.launch.py` should be used for robot bringup in other launch files.

## 🥃🧊🍒 View Carrie
To visualize carrie you can use
```shell
ros2 launch carrie_description view_carrie.launch.py
```
which will open an rviz window and allow you to inspect meshes and tf tree.

<p align="center"> <img src="docs/rviz_carrie.png" alt="rviz carrie" width="1000"/> </p>

