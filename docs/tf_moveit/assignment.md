## Assignment

At the end of this assignment you will have a set of tools that you can use during the practical part of the workshop. So do your best to make it as useful and easy-to-use as possible.

### First part of the assignment
Using the joint state publisher and a simulated robot save various joint and Cartesian space configurations into a YAML file

Example of YAML file obtained with this:
```yaml
```

### Second part of the assignment
Using the YAML that you created in the first part of the assignment you will publish the message that were stored as `TransformStamped` onto TF.

Next, you will move the simulated robot into these configurations, joint and Cartesian space.

Example code that moves the robot into a joint space configuration that you previously called `home`:
```python
...
move_group.go(saved_configurations['home'])
...
```