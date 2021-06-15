# coppelia_ros_pioneer
Tested on Ubuntu 18.04 and Coppelia 4.1.0.

## Prerequesites
This package relies on some other packages and dependencies to work properly and do SLAM.

### CoppeliaSim
This will only work on CoppeliaSim. We included the scenes we used for testing. To use your own scenes, you need to copy the pioneer model to your scene.
https://www.coppeliarobotics.com/downloads

### robot_localization package
Kalman Filter to fuse IMU data. https://github.com/cra-ros-pkg/robot_localization 

### Convert PointCloud PointCloud2
The Hokuyo laser used in CoppeliaSim publishes data in PointCloud format. We use this package to convert to PointCloud2 (actually, it's a fork with a small modification). https://github.com/leohmcs/point_cloud_converter

### Cartographer ROS, used for SLAM
This package includes files to configure and launch Cartographer for 2D SLAM. The instructions to run it are below. Of course, if you want to use it, you need to install Cartographer first. https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html

### Convert PointCloud2 to LaserScan
We configured Cartographer to use LaserScan data for mapping. https://github.com/ros-perception/pointcloud_to_laserscan

### Robot Model to show in RViz
To make the visualization more exciting, you can display this model on RViz. It is a fork with a small modification to include the IMU. https://github.com/leohmcs/p3dx

## Overview
This package provides components to give basically functionalities to a Pioneer3dx simulated in CoppeliaSim. It provides the robot odometry (wheel encoder + IMU), the possibility to control the robot using your keyboard and the data from a Hokuyo scan, which we use for SLAM. It also provides a simple TurnAndGo algorithm, so you can give points for the robot to follow.

You can also run two instances of the Pioneer simultaneously, both for mapping and TurnAndGo. 

## Mapping
As mentioned before, we use Cartographer for SLAM.

### One robot
Load the scene `ros_pioneer.ttt`, included in `coppelia_scenes` directory of this package, into CoppeliaSim. This scene has the robot in a medium sized scenario, which makes an insteresting map. Once the scene is loaded, start the simulation and run

```
roslaunch coppelia_ros_pioneer pioneer_2d.launch
```
### Two robots
You can also use `ros_pioneer.ttt`, but you will have to remove the Pioneer that is there by default and add the models from `coppelia_scenes/two_robots` directory. Once the scene is loaded and you added the models, run

```
roslaunch coppelia_ros_pioneer pioneer_map_group.launch
```

(of course, ROS Master must be running in both cases)

## TurnAndGo
### One robot
The simulator is optional here. You can use the same robot model used for SLAM. In this case, RViz provides a great visualization. To run the algorithm, uncomment the indicated lines in `pioneer.launch`. Then run

```
roslaunch coppelia_ros_pioneer pioneer.launch
```

The points are set as an `arg` in the same file.

To visualize in RViz

```
roslaunch coppelia_ros_pioneer turn_and_go_single.rviz
```

### Two robots
To run for two robots, you also need to uncomment the indicated lines in `pioneer.launch`. Then run

```
roslaunch coppelia_ros_pioneer rviz_turn_and_go_single.launch
```

The points are set as an `arg` in the same file (`pioneer_turn_and_go_group.launch`).

To visualize in RViz

```
roslaunch coppelia_ros_pioneer rviz_turn_and_go_two.launch
```
