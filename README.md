# coppelia_ros_pioneer
Tested on Ubuntu 18.04 and Coppelia 4.1.0.

## To run
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
This will run odometry.py, which calculates odometry based on wheel encoder, wheel_velocity.py, which calculates each wheel velocity based on Twist messages received from /cmd_vel (published by teleop node), and static_transform_publisher node to calculate base_link -> imu transform.

```
roslaunch coppelia_ros_pioneer odom_ekf.launch
```
This will setup and run [robot_localization](http://wiki.ros.org/robot_localization)'s ekf_localization_node to fuse imu and wheel encoder odometry data to improve odometry precision. This node also publishes odom -> base_link tf.

Run teleop to move the robot using your keyboard. The source code can be found at [https://github.com/ros-teleop/teleop_twist_keyboard](#)
```
# If you don't have the package insalled, clone the repository to your workspace and build
git clone git@github.com:ros-teleop/teleop_twist_keyboard.git
catkin build

# To run teleop node
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
``` 

Refer to the [repository](https://github.com/ros-teleop/teleop_twist_keyboard) for more information and usage instructions.

## RViz
The model used on RViz is [https://github.com/mario-serna/pioneer_p3dx_model](#)

To install
```
git clone git@github.com:mario-serna/pioneer_p3dx_model.git
catkin build
```

To use
```
roslaunch p3dx_description rviz.launch
```
