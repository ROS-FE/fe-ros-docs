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
cd ~/catkin_ws/src
catkin_create_pkg rpi_feros rospy std_msgs actionlib_msgs
catkin_make
```

## Node
Nodes are processes:
- used for calculations,
- that run inside robotic application,
- grouped into packages,
- that communicate with each other (topics, servers, actions, parameter servers). 

Why to use nodes:
- reduce code complexity,
- the code is more error-resistant,
- use of different programming languages.

### New node

Nodes are scripts that are located inside `scripts` folder.
```
roscd rpi_feros
mkdir scripts
cd scripts
```
Create new script `my_first_node.py`:
```
touch my_first_node.py
chmod +x my_first_node.py
```

Open script with Visual Studio Code:
```
code my_first_node.py
```

The minimal working example for node:

```python linenums="1"
#!/usr/bin/env python

import rospy

if __name__ == '__main__':
  rospy.init_node('my_first_python_node')
  rospy.loginfo('This node has been started.')
  rospy.sleep(1)
  print('Exit now')
```



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
