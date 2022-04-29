# Introduction to ROS

Welcome to the ROS FE workshop. In this part of the workshop you will learn the fundamentals of ROS.

## Why ROS?

Why, oh why????

## ROS setup

### roscore
- takes care of communicaiton between different ROS functionalities
- it can run only one at once
- connects different ROS system into one ROS network

```python
roscore
```

### catkin workspace

CATKIN is an official build system for ROS

```
cd
mkdir catkin_ws
cd catkin_ws
mkdir src
catkin_make
```

Alternative is to use `catkin build` (beforehand you need to remove devel and build folders).

Conect console with ROS variables
```
cd devel
source setup.bash
```

To do this automatically when the console is opened, add to `bashrc.sh`
```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Packages

Packages are independent units, that can be re-used.

### New package

Go to `catkin_ws/src/` folder.

Basic syntax:
```
catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
```

Create new package `rpi_feros`:

```
roscd
cd ..
cd src
catkin_create_pkg rpi_feros rospy std_msgs actionlib_msgs
catkin_make
```

## Node

## Topics

### Exercise

## Services

### Exercise

## Custom messages

### Exercise

## Parameters

## Launch files

### Exercise

## ROS network

## Action server

### Exercise
