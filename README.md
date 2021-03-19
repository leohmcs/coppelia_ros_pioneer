# coppelia_ros_pioneer
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
