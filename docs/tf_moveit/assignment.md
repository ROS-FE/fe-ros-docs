## Assignment

At the end of this assignment you will have a set of tools that you can use during the practical part of the workshop. So do your best to make it as useful and easy-to-use as possible.

### First part of the assignment
Using the joint state publisher and a simulated robot save various joint and Cartesian space configurations into a Pick file. Use the scripts that you wrote in the previous assignment.


### Second part of the assignment
Using the Pickle file that you created in the first part of the assignment you will publish the message that were stored as `TransformStamped` onto TF. You will do so using the script that you write in the previous assignments.

Next, you will move the simulated robot into these configurations. Help yourselves with the convenient methods: `transformstamped_to_pose` and `get_pose_from_tf`. You will find the code snippets below.

#### `transformstamped_to_pose`
```python
def transformstamped_to_pose(transform_stamped):
    """Convert from geometry_msgs/TransformStamped to geometry_msgs/Pose

    Args:
        transform_stamped (geometry_msgs/TransformStamped): The geometry_msgs/TransformStamped message to convert

    Returns:
        Pose: The geometry_msgs/Pose message
    """
    return Pose(position=transform_stamped.transform.translation, orientation=transform_stamped.transform.rotation)
```

#### `get_pose_from_tf`

```python
def get_pose_from_tf(target, tf_buffer, relative_to='base_link'):
    """Return a Pose object to the desired TF frame

    Args:
        target (str): TF frame that we are interested in
        tf_buffer (tf2_ros.Buffer): A TF buffer. Don't forget to initialize it with the listener!!!
        relative_to (str, optional): Relative to ...

    Return:
        Pose: The TF frame expressed as geometry_msgs/Pose
    """

    rospy.sleep(0.1)
    target_transform = tf_buffer.lookup_transform(target, relative_to, rospy.Time(0))
    return transformstamped_to_pose(target_transform)
```