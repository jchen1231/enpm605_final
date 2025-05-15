# ENPM605_FinalProject_Group4

## Contact Info (Just in Case)

- I am always online
- (443) 676-4298
- <jasonchen4298@gmail.com>

## System Info

- Ubuntu 22.04.5 LTS
- Python 3.10.12 64-bit
- ROS2 humble
- Ignition Gazebo 6 - Fortress

## Building the Packages

- Source ROS 2
- Go into your source folder: cd /your_workspace/src
- Build the provided simulation environment:

```bash
colcon build --symlink-install --packages-up-to rosbot --cmake-args -DCMAKE_BUILD_TYPE=Release
```

- Build the provided Python demo packages and my packages (custom_interfaces and final):

```bash
colcon build --symlink-install --packages-select ros2_aruco_interfaces ros2_aruco custom_interfaces final
```

- Source setup file

## Packages Used

- Below are the specific versions that worked for me:
  - opencv-contrib-python==3.4.18.65
  - transforms3d==0.4.2
  - setuptools==58.0.0
  - tf-transformations==1.1.0
  - PyYAML==5.4.1
  - numpy==1.22.4
- The main dependencies used:
  - nav2_bringup
  - nav2_amcl
  - nav2_simple_commander
  - nav2_bringup
  - geometry_msgs
  - rclpy
  - launch
  - launch_ros

- Install missing all dependencies at once (I added everything to the package.xml file):

```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## Launch

- Use the launch file navigation.launch.py:

```bash
ros2 launch final navigation.launch.py
```

## Customize map

- To change camera's pose, need to modify both the *.sdf file and the launch parameter in the file navigation.launch.py
- To change cubes' orientations, just need to modify *.sdf file
- Can change robot starting position as well
