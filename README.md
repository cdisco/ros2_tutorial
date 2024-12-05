# ros2_tutorial
The files in this project are a result of following a Udemy ROS 2 tutorial

Note, my stating point was a ROS 2 jazzy desktop container rather than the virtual machine recommended in the tutorial itself.
```
docker run -it osrf/ros:jazzy-desktop bash
```

# Getting started

## Building
Either mount or copy all the source files into a directory within the docker container.
Then build one or all of the packages
```
colcon build
```
or
```
colcon build --packages-select <package_name>
```
## Running nodes

Plenty of online documentation regarding how to run nodes but here's one
https://design.ros2.org/articles/ros_command_line_arguments.html
