# Practical work with robots

Welcome to the practical part of the workshop.

## FE ROS Application Programming Interface

We have prepared a couple of helpful methods, classes and scripts to give you a little bit of a head-start with programming the robot motions.

### Helpful motion methods

In the [repository](https://github.com/ROS-FE/fe_ros_workshop/) that you have cloned into your workspace there is a package called `fe_ros`. We provide some helpful Python modules that can be imported into your scripts. One of them is `fe_moveit`. It's a wrapper around the `MoveGroupCommander` and using it is fairly easy. Here is an example:

```python
#!/usr/bin/env python2

# Import the MoveGroupCommander class from the fe_moveit module
from fe_moveit import MoveGroupCommander

# Create a move group
move_group = MoveGroupCommander('manipulator')

# Define a target list in joint space
target_joints = [0, 0, 1.57, 0, 0, 0]

# Send the target to MoveIt and see the robot follow a joint space trajecotry
move_group.movej(target=target_joints, wait=True, speed=0.5)
```

Naturally, `movej` is just one of the methods we provide. The following is a full list of the methods we provide for your convenience:

* `movej` - Will move the robot in joint space following the trapezoidal speed profile
* `movel` - Will move the robot to the Cartesian space target following a straight line
* `movej_relative` - Will move the robot in joint space with an offset to a defined frame
* `movel_relative` - Will move the robot in Cartesian space following a straight line with an offset to a defined frame.

For a more detailed explanation open the Python module where these methods are defined.


### Helpful stop service

The modified `MoveGroupCommander` also starts a service that will stop the robot's motion when called. The service is of type `std_srv/Empty` and is available at `/interrupt_motion`.