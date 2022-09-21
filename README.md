# ROS `loomo_base` package
🤖 Segway Ninebot Loomo ROS control


## Build
Use the following commands to download and compile the package. [Catkin_tools](https://catkin-tools.readthedocs.io/en/latest/installing.html) is recommended, but catkin_make also works.
```
cd catkin_ws/src/
git clone https://github.com/jkk-research/loomo_base
catkin build loomo_base
```
Don't forget to source 😉

## Run 
```
roslaunch loomo_base joystick_control.launch
```

# Troubleshoot
```
sudo apt-get install ros-melodic-joy
```

## Further readings 
- https://github.com/mit-acl/android_loomo_ros_core

![](etc/loomo01.svg)
