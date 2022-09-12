## MoveIt

In this section we will learn how to plan and execute robot trajectories using MoveIt.

### What is MoveIt?

From their [Github repository](https://github.com/ros-planning/moveit):
> Easy-to-use open source robotics manipulation platform for developing commercial applications, prototyping designs, and benchmarking algorithms.

The best way to show why MoveIt is cool and how you can use it is by providing a demo. We've prepared a simple launch file that starts Rviz (you will learn about this later) with the MoveIt plug in:
```
$ roslaunch fe_ur urko_moveit_demo.launch
```

<!-- The instructor shows a demo running moveit with `roslaunch fe_ros urko_moveit_demo.launch`  -->

### Understanding the MoveIt architecture 

> **Note**: The images displayed in this section were taken from the documentation avaialbe at https://moveit.ros.org/documentation/concepts/

![MoveIt architecture](images/moveit_scheme.png)

#### The Move Group

This is the node that is the central program that combines all the available environment data and user requests in order to provide a trajectory that leads the robot from one configuration to the next by using the available motion planners.

#### Robot description
The robot you saw in the demo before was defined in a URDF file (with a combination of [XACRO](http://wiki.ros.org/xacro)). This defines the model of the robot, the parts that we don't expect to change beyond the normal motions of its actuators.

#### Semantic description
The URDF file might describe a system with hundreds of frames. The so called semantic description (a `.srdf` file) tells the Move Group which are the frames that constitute our robot. Moreover, it tells what object pairs (i.e. frames with some shape attached to them) are to be ignored when checking collisions.

#### The environment

MoveIt is aware of the robot's environment while planning trajectories. This environment can be divided in two groups: the URDF (i.e. robot description, what we know) and the planning scene (what we detect).


> The instructor show how to add a cube into the planning scene through the user interface

### MoveIt Python API

Playing with a graphical interface to see how a robot can move in an environment is fun but not too useful for solving practical programs. We are robot programmers, which means we want to write robot programs. This is precisely the aim of this section - explore the Python API and how can it be used to program robot motions.

Let's start with the following code snippet:
```python
#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import JointState

JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint'
]

HOME_JOINTS = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]

if __name__ == '__main__':
    rospy.init_node('moveit_programmer')

    joint_pub = rospy.Publisher('/move_group/fake_controller_joint_states', JointState, queue_size=10)
    rospy.sleep(0.1)

    init_joints = JointState()
    init_joints.header.stamp = rospy.Time.now()
    init_joints.name = JOINT_NAMES
    init_joints.position = HOME_JOINTS
    joint_pub.publish(init_joints)
    #########################
    ##### STUDENT WRITES ####
    #########################


    #########################
```

This code snippet publishes the initial joint configuration on the `/move_group/fake_controller_joint_states` topic and then quits. The `joint_state_publisher` node reads the values published on this topic and copies them. Which means, that whenever we start this node, our simulated robot will be in some sort of initial position.

Paste this code snippet in a file called `moveit_programmer.py` and make it executable (`chmod +x ...`). We are now going to write the code in the marked section of the code.

#### Simple motion to target joint positions

To start using MoveIt in our Python code, we must first import it: `import moveit_commander`. Next, we need to create an instance of the `MoveGroupCommander`:
```python
...
    moveit_interface = moveit_commander.MoveGroupCommander('manipulator')
...
```

We can now use the full range of the MoveIt's Python API. Let's try something simple: get the current values of the robot's joints:
```python
...
    joint_goal = moveit_interface.get_current_joint_values()
    print(joint_goal)
...
```

The `get_current_joint_values` method returns the current joint values (duh) as a Python list. If you have added these three lines into the `moveit_programmer.py` script and executed it, you should see the following print to the standard output:
```
$ rosrun  fe_tf moveit_programming.py 
[ INFO] [1663002388.870181400]: Loading robot model 'ur10e_robot'...
[ INFO] [1663002390.056249500]: Ready to take commands for planning group manipulator.
[6.663211309351027e-05, -1.0706547763434724, -1.875588055700064e-05, -1.5700763391067274, 9.441151451319457e-05, 6.011523539200426e-05]
```

Let's modify the 2nd element in this list by changing its sign and instruct MoveIt to move the robot there:
```python
    joint_goal[1] = - HOME_JOINTS[1]
    moveit_interface.go(joint_goal, wait=True)
```
If all goes well, you should see the virtual robot move. Yay!

### First assignment

Write a program in Python that will guide the robot from ... to ... without collisions using MoveIt Python API.


