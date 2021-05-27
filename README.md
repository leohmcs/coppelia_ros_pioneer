# coppelia_ros_pioneer
Tested on Ubuntu 18.04 and Coppelia 4.1.0.

## Prerequesites
### Minimum
Simulator, CoppeliaSim: https://www.coppeliarobotics.com/downloads

Kalman Filter to fuse IMU data, robot_localization package: https://github.com/cra-ros-pkg/robot_localization

Convert PointCloud data from CoppeliaSim to PointCloud2: https://github.com/pal-robotics-forks/point_cloud_converter

Cartographer ROS, used for SLAM: https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html

ConvertPointCloud2 to LaserScan: https://github.com/ros-perception/pointcloud_to_laserscan (you can also use PointCloud2 directly on Cartographer, if you wish).

Finally, Robot Model to show in RViz: https://github.com/mario-serna/pioneer_p3dx_model/tree/master/p3dx_description/urdf

## 1) To run
Clone and build the repository in your workspace

```
# In the first level of your workspace
git clone git@github.com:leohmcs/coppelia_ros_pioneer.git
catkin build
```

Launch the odometry and wheel_velocity nodes

```
roslaunch coppelia_ros_pioneer pioneer.launch
```

This will run odometry.py, which calculates odometry based on wheel encoder, wheel_velocity.py, which calculates each wheel velocity based on Twist messages received from /cmd_vel (published by teleop node).

```
roslaunch coppelia_ros_pioneer odom_ekf.launch
```

This will setup and run [robot_localization](http://wiki.ros.org/robot_localization)'s ekf_localization_node to fuse imu and wheel encoder odometry data to improve odometry precision. This node also publishes odom -> base_link tf.

Run teleop to move the robot using your keyboard. The source code can be found at [https://github.com/ros-teleop/teleop_twist_keyboard](https://github.com/ros-teleop/teleop_twist_keyboard)

```
# If you don't have the package insalled, clone the repository to your workspace and build
git clone git@github.com:ros-teleop/teleop_twist_keyboard.git
catkin build

# To run teleop node
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
``` 

Refer to the [repository](https://github.com/ros-teleop/teleop_twist_keyboard) for more information and usage instructions.

## 2) RViz
The model used on RViz is [https://github.com/mario-serna/pioneer_p3dx_model](https://github.com/mario-serna/pioneer_p3dx_model)

To install

```
git clone git@github.com:mario-serna/pioneer_p3dx_model.git
catkin build
```

To use

```
roslaunch p3dx_description rviz.launch
```
## 3) Cartographer
### Installation
I recommend following the official tutorial to install Cartographer ROS.
https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html

### One Robot
Once you have installed Cartographer, you will need two files to run Cartographer SLAM: `pioneer_2d.lua` and `pioneer_2d.launch`. The first one configures some parameters, such as wether Cartographer will be responsible for publishing Odom frame or not. These are the parameters you will change to tune the mapping. The second one launchs RViz for visualization.

```
roslaunch coppelia_ros_pioneer pioneer_2d.lua
```

This command should work. If it doesn't, copy the files to Cartographer respective directories.

```
# Assuming you have installed coppelia_ros_pioneer and cartographer_ros (following the recommended tutorial) packages in a workspace called workspace.
cp ~/workspace/src/coppelia_ros_pioneer/launch/pioneer_2d.launch ~/workspace/install_isolated/share/cartographer_ros/launch/
cp ~/workspace/src/coppelia_ros_pioneer/configuration_files/pioneer_2d.lua ~/workspace/install_isolated/share/cartographer_ros/configuration_files/
```

And then run

```
source ~/workspace/install_isolated/setup.bash  # use setup.zsh if your shell is zsh
roslaunch cartographer_ros pioneer_2d.launch
```
### Two Robots

## TurnAndGo
A simple "turn and go" algorithm was recently added to the package. It was tested up to two robots running simultaneously, although it may work for more with a few changes on the launch file (basically, add more `<group>` and `namespaces`).

### One Robot
With odometry properly running (see Section 1), you just need to add the points you want the robot to follow as an argument (`arg pts`) in the `pioneer.launch` file. Then run

```
roslaunch coppelia_ros_pioneer pioneer.launch
```

### Two Robots
In order to run "turn_and_go" simultaneously for two robots, each one following a different list of points, run

```
roslaunch coppelia_ros_pioneer pioneer_turn_and_go_group.launch
```
 There is a template in the same file (`pioneer_turn_and_go_group.launch`) to configure each robot, i.e. set namespaces, tf_prefix and the list of points.
