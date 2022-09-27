## Transformations

In the first part of this day's workshop, we will learn how can we publish, read and visualize transformations.

### What is TF?

From [TF's wiki page](http://wiki.ros.org/tf):
> tf is a package that lets the user keep track of multiple coordinate frames over time.

In this part of the workshop, you will learn how we can use TF in practice and what needs to be taken care of.

#### Where's the information

The transformation data between the various frames is available across two different topics: `/tf_static` and `/tf`.

As the name suggests, the `/tf_static` topic holds the transformation data between frames that are, well, static! Whereas `/tf` holds the transformation data between frames which relations change. Let's learn how the two of them differ from one another in practice!

### TF static publisher

The static publisher is very useful to publish spatial transformation data that we don't expect to change. In robotics that might be the relation between the robot's base and a fixed point in it's proximity. For example, the transformation between the robot's base and the origin of the table that the robot is mounted on.

#### Command line demonstration

Open three terminals. In the first terminal, run the following command:
```
$ static_transform_publisher 0 0 1 0 0 0 world frame_1 1
```

In the second terminal, run the following command:
```
$ static_transform_publisher 0 0 2 0 0 0 frame_1 frame_2  1
```

Leave this two terminals open and switch to the third.

Let's inspect the content of the `/tf_static` topic:
```
$ rostopic echo /tf_static
```
What do you see? How many messages did you receive? Are they periodic?

Ok, now let's retrieve the transformation between `world` and `frame_2`. What do you expect to be the result?
```
rosrun tf tf_echo world frame_2
```

> **Note**: The `tf_echo` tool is very useful to evaluate the transformation between two frames on TF.
#### Python demonstration

Take the following code snipped and let's dig in:

```python
#! /usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

if __name__ == '__main__':

    rospy.init_node('tf_static_broadcaster')

    tf_broad = tf2_ros.StaticTransformBroadcaster()

    frame_1 = TransformStamped()
    frame_1.child_frame_id = 'frame_1'
    frame_1.header.frame_id = 'world'

    frame_1.transform.rotation.w = 1.0

    frame_1.transform.translation.z = 1.0
    frame_1.transform.translation.y = 0.0
    frame_1.transform.translation.x = 0.0

    rospy.loginfo("Publishing transform from {0} to {1}".format(frame_1.header.frame_id, frame_1.child_frame_id))
    tf_broad.sendTransform([frame_1])

    rospy.spin()
```

#### Intermediate assignment

Write a Python script that publishes three frames: `world`, `frame_1` and `frame_2`. The relations between them should be as follows:
- `frame_1` is offset by 1m along the `Z` axis from `world`
- `frame_2` is offset by 1m along the `X` axis from `frame_1`

Check with `tf_echo` the relation between `world` and `frame_2`. You should see the following output in the command line:
```
At time 0.000
- Translation: [1.000, 0.000, 1.000]
- Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
            in RPY (radian) [0.000, -0.000, 0.000]
            in RPY (degree) [0.000, -0.000, 0.000]
```

### TF publisher

When we want to publish transformation that we know change often, we rather use the "normal" TF publisher. This means, not static.

This type of publisher does not have a command line interface. That's why we will dig into a Python example right away.

#### Python demonstration

To start with, let's use the previous script and instead of using `StaticTransformBroadcaster()` we use `TransformBroadcaster()`.

After you modified the file run the script. Observe the contents of `/tf_static` and `/tf` topic. Seeing any messages?

The reason why you don't see any messages is because `TransformBroadcaster()` publishes to `/tf`, which is not a **latched** topic. Your script published exactly one message to `/tf` and then got stuck in the `rospy.spin()`. To confirm that, subscribe to the `/tf` topic again and start the script, you will see just one message.
> Note: You might need to add `rospy.sleep(0.1)` just before sending the transform. Sometimes, the `TransformBroadcaster()` needs some time to set everything up.

Let's now write a script, that publishes the frame with a constant frequency:
```python
#! /usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

if __name__ == '__main__':

    rospy.init_node('tf_broadcaster')

    tf_broad = tf2_ros.TransformBroadcaster()

    frame_1 = TransformStamped()
    frame_1.child_frame_id = 'frame_1'
    frame_1.header.frame_id = 'world'

    frame_1.transform.rotation.w = 1.0

    frame_1.transform.translation.z = 1.0
    frame_1.transform.translation.y = 0.0
    frame_1.transform.translation.x = 0.0

    while not rospy.is_shutdown():
        tf_broad.sendTransform([frame_1])
        rospy.sleep(0.01)

```

As you see, we no longer require the `rospy.spin()`. That is because we use a `while` loop that does not exit except if we interrupt the execution of the Python script.

Let's inspect the TF topics. As you expect, the `/tf_static` is not publish anything. On the other hand, the `/tf` is publishing a lot of messages:
```
$ rostopic hz /tf
subscribed to [/tf]
average rate: 95.603
        min: 0.009s max: 0.011s std dev: 0.00031s window: 9
```
Approximately 100 Hz.

#### Intermediate assignment

Write a Python script that publishes three frames: `world`, `frame_1` and `frame_2`. The relations between them should be as follows:
- `frame_1` is offset by 1m along the `Z` axis from `world`
- `frame_2` is offset by `sin(t)` along the `X` and `cos(t)` along the `Z` axis from `frame_1`

