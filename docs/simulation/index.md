# ROS and simulation

## Gazebo

Gazebo documentation https://classic.gazebosim.org/

Gazebo tutorials https://classic.gazebosim.org/tutorials?cat=guided_b&tut=guided_b1

From Gazebo documentation: 
- Gazebo is a 3D dynamic simulator for simulating robots, environments and sensors. 
- Gazebo offers physics simulation, typicaly much better than simulators in game engines. It supports various physical engines: ODE, Bullet, Simbody, DART.
- Gazebo is typical used for testing robotics algorithms in realistic scenarios.
- Gazebo has repository of 3D models: http://models.gazebosim.org/.
- Gazebo has good support:
  - documentation, 
  - tutorials,
  - use cases and examples.
 
A few key features of Gazebo include:
- multiple physics engines,
- a rich library of robot models and environments,
- a wide variety of sensors,
- convenient programmatic and graphical interfaces.

Why use Gazebo? Simulation of robots is fundamental part of developement and testing of applications and algorithms.

Gazebo can be run in terminal with command
```
gazebo
```

See tutorial http://gazebosim.org/tutorials?tut=quick_start

### Gazebo GUI

https://classic.gazebosim.org/tutorials?cat=guided_b&tut=guided_b2

### 2 DOF robot visualization

Lets first create a new catkin workspace.

```
mkdir catkin_ws
cd catkin_ws
mkdir src
catkin_make
source devel/setup.bash
```

Lets create new package for simple 2 DOF robot.

```
cd src
catkin_create_pkg robot_2dof roscpp rospy std_msgs
```

We will create 2 folders:
- folder `urdf` for storing urdf files and
- folder `launch` for launch files.

```
cd robot_2dof
mkdir urdf
mkdir launch
cd ../..
catkin_make
```

Check if required packages are installed with command 

```
rospack list-names
```

We will require packages

```
urdf_tutorial
robot_state_publisher
joint_state_controller
joint_state_publisher_gui
```

We will create urdf file for 2 DOF robot. Move into catkin workspace folder and then into `urdf` folder.

```
cd src/robot_2dof/urdf/
```

Create urdf file and open it in editor.

```
touch robot2dof.urdf
code robot2dof.urdf
```

Copy the code into urdf file.

```xml
<?xml version="1"?>
<robot name="2dof_robot">

    <material name="White">
        <color rgba="1.0 1.0 1.0 1.0"/>
    </material>

    <material name="Blue">
        <color rgba="0.0 0.0 1.0 1.0"/>
    </material> 

    <material name="Red">
        <color rgba="1 0 0 1.0"/>
    </material>

    <material name="Black">
        <color rgba="0 0 0 1.0"/>
    </material>    

   <link name="world"/>
   
   <!-- BASE LINK-->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.16 0.16 1.0"/>
            </geometry>
            <material name="White"/>
            <origin xyz="0.0 0.0 0.5" rpy="0.0 0.0 0.0"/>
        </visual>
        <collision>
            <geometry>
                <box size="0.16 0.16 1.0"/>
            </geometry>
            <origin xyz="0.0 0.0 0.5" rpy="0.0 0.0 0.0"/>
        </collision>
    </link>

    <joint name="fixed_base" type="fixed">
        <parent link="world"/>
        <child link="base_link"/>
    </joint>

    <!-- PRVI SEGMENT-->
    <link name="prvi_segment">
        <visual>
            <geometry>
                <cylinder radius="0.04" length="0.5"/>
            </geometry>
            <material name="Red"/>
            <origin xyz="0.12 0.0 0.21" rpy="0.0 0.0 0.0"/>
        </visual>
        <collision>
            <geometry>
                <cylinder radius="0.04" length="0.5"/>
            </geometry>
            <origin xyz="0.0 0.0 0.21" rpy="0.0 0.0 0.0"/>
        </collision>
    </link>

    <joint name="prvi_sklep" type="continuous">
        <axis xyz="1 0 0"/>
        <parent link="base_link"/>
        <child link="prvi_segment"/>
        <origin xyz="0.0 0 0.92" rpy="-1.5707 0 0"/>
    </joint>

     <!-- DRUGI SEGMENT-->
    <link name="drugi_segment">
        <visual>
            <geometry>
                <cylinder radius="0.04" length="0.5"/>
            </geometry>
            <material name="Blue"/>
            <origin xyz="-0.08 0.0 0.21" rpy="0.0 0.0 0.0"/>
        </visual>
        <collision>
            <geometry>
                <cylinder radius="0.04" length="0.5"/>
            </geometry>
            <origin xyz="0.0 0.0 0.21" rpy="0.0 0.0 0.0"/>
        </collision>
    </link>

    <joint name="drugi_sklep" type="continuous">
        <axis xyz="1 0 0"/>
        <parent link="prvi_segment"/>
        <child link="drugi_segment"/>
        <origin xyz="0.12 0 0.42" rpy="0 0 0"/>
    </joint>

        <!-- TCP SEGMENT-->
    <link name="TCP_segment">
        <visual>
            <geometry>
                <sphere radius="0.02"/>
            </geometry>
            <material name="Red"/>
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
        </visual>
        <collision>
            <geometry>
                <sphere radius="0.02"/>
            </geometry>
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
        </collision>
    </link>

    <joint name="TCP_sklep" type="fixed">
        <parent link="drugi_segment"/>
        <child link="TCP_segment"/>
        <origin xyz="-0.08 0 0.46" rpy="0 0 0"/>
    </joint>

</robot>
```

We can visualize the model in rviz with command.

```
roslaunch urdf_tutorial display.launch model:='$(find robot_2dof)/urdf/robot2dof.urdf'
```

![Image shows rviz and gui for 2 DOF robot](https://drive.google.com/file/d/1X54P4-nOAK6FIVNT1NZYWWqXSZYkcdr7/view?usp=sharing)
![Image shows rviz and gui for 2 DOF robot](https://1drv.ms/u/s!AhNeN6Tj2T_kbee4kGdFA8JOFgA?e=gLcmE0)

In new terminal window we can list all the ROS topics with command
```
rostopic list
```
We receive output
```
/clicked_point
/initialpose
/joint_states
/move_base_simple/goal
/rosout
/rosout_agg
/tf
/tf_static
```

We can print the values of the joints in terminal window with command
```
rostopic echo /joint_states
```

Now if we change the values using slider on the GUI we can follow the values of joints.



### Visualization of Franka Panda robot

We will now visualize more complex model of the robot, such as Franka Panda robot.

Copy file `panda.zip` into folder `urdf` and unzip it. 

Run command 

```
roslaunch urdf_tutorial display.launch model:='$(find robot_2dof)/urdf/panda.urdf'
```

In rviz you need to change *Fixed Frame* (in *Global Options*) to *panda_link0*.

### Importing 2 DOF robot in gazebo

First, we will create launch file to load urdf file into gazebo.

```xml
<?xml version="1.0"?>

<launch>

    <param name="robot_description" textfile="$(find robot_2dof)/urdf/robot2dof.urdf" />
    <include file="$(find gazebo_ros)/launch/empty_world.launch" />
   
    <node pkg="gazebo_ros" name="spawn_urdf" type="spawn_model" args="-param robot_description -urdf -model robot2dof" />
   
    <!-- -->

</launch>
```

We can run the launch file with command 
```
roslaunch robot_2dof zagon_gazebo.launch
```

What we see is only empty gazebo world without the robot. This is because the urdf file does not contain everything we need for simulation in gazebo. First, we need to add `<inertial>` tag to each link. Below is urdf file with added inertial parameters for each link.

```xml
<?xml version="1"?>
<robot name="2dof_robot">

    <material name="White">
        <color rgba="1.0 1.0 1.0 1.0"/>
    </material>

    <material name="Blue">
        <color rgba="0.0 0.0 1.0 1.0"/>
    </material> 

    <material name="Red">
        <color rgba="1 0 0 1.0"/>
    </material>

    <material name="Black">
        <color rgba="0 0 0 1.0"/>
    </material>    

   <link name="world"/>
   
   <!-- BASE LINK-->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.16 0.16 1.0"/>
            </geometry>
            <material name="White"/>
            <origin xyz="0.0 0.0 0.5" rpy="0.0 0.0 0.0"/>
        </visual>
        <collision>
            <geometry>
                <box size="0.16 0.16 1.0"/>
            </geometry>
            <origin xyz="0.0 0.0 0.5" rpy="0.0 0.0 0.0"/>
        </collision>
        <inertial>
            <mass value="25"/>
            <origin xyz="0.0 0.0 0.5"/>
            <inertia ixx="8.741" iyy="8.741" izz="0.4367" ixy="-0.1637" ixz="-0.1023"  iyz="-0.1023"/>
        </inertial>
    </link>

    <joint name="fixed_base" type="fixed">
        <parent link="world"/>
        <child link="base_link"/>
    </joint>

    <!-- PRVI SEGMENT-->
    <link name="prvi_segment">
        <visual>
            <geometry>
                <cylinder radius="0.04" length="0.5"/>
            </geometry>
            <material name="Red"/>
            <origin xyz="0.12 0.0 0.21" rpy="0.0 0.0 0.0"/>
        </visual>
        <collision>
            <geometry>
                <cylinder radius="0.04" length="0.5"/>
            </geometry>
            <origin xyz="0.0 0.0 0.21" rpy="0.0 0.0 0.0"/>
        </collision>
        <inertial>
            <mass value="2.5"/>
            <origin xyz="0.0 0.0 0.25"/>
            <inertia ixx="0.2091" iyy="0.2091" izz="0.002" ixy="0.0" ixz="0.0"  iyz="0.0"/>
        </inertial>
    </link>

    <joint name="prvi_sklep" type="continuous">
        <axis xyz="1 0 0"/>
        <parent link="base_link"/>
        <child link="prvi_segment"/>
        <origin xyz="0.0 0 0.92" rpy="-1.5707 0 0"/>
    </joint>

     <!-- DRUGI SEGMENT-->
    <link name="drugi_segment">
        <visual>
            <geometry>
                <cylinder radius="0.04" length="0.5"/>
            </geometry>
            <material name="Blue"/>
            <origin xyz="-0.08 0.0 0.21" rpy="0.0 0.0 0.0"/>
        </visual>
        <collision>
            <geometry>
                <cylinder radius="0.04" length="0.5"/>
            </geometry>
            <origin xyz="0.0 0.0 0.21" rpy="0.0 0.0 0.0"/>
        </collision>
        <inertial>
            <mass value="2.5"/>
            <origin xyz="0.0 0.0 0.25"/>
            <inertia ixx="0.2091" iyy="0.2091" izz="0.002" ixy="0.0" ixz="0.0"  iyz="0.0"/>
        </inertial>
    </link>

    <joint name="drugi_sklep" type="continuous">
        <axis xyz="1 0 0"/>
        <parent link="prvi_segment"/>
        <child link="drugi_segment"/>
        <origin xyz="0.12 0 0.42" rpy="0 0 0"/>
    </joint>

        <!-- TCP SEGMENT-->
    <link name="TCP_segment">
        <visual>
            <geometry>
                <sphere radius="0.02"/>
            </geometry>
            <material name="Red"/>
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
        </visual>
        <collision>
            <geometry>
                <sphere radius="0.02"/>
            </geometry>
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
        </collision>
        <inertial>
            <mass value="0.034"/>
            <origin xyz="0.0 0.0 0.0"/>
            <inertia ixx="0.0000054" iyy="0.0000054" izz="0.0000054" ixy="0.0" ixz="0.0"  iyz="0.0"/>
        </inertial>
    </link>

    <joint name="TCP_sklep" type="fixed">
        <parent link="drugi_segment"/>
        <child link="TCP_segment"/>
        <origin xyz="-0.08 0 0.46" rpy="0 0 0"/>
    </joint>

</robot>
```

Simulation starts and the robot acts as 2 DOF pendulum. We can add a *pause* argument in lauch file, so that the simulation does not run automatically.

```xml
<?xml version="1.0"?>

<launch>

    <arg name="paused" default="true"/>

    <param name="robot_description" textfile="$(find robot_2dof)/urdf/robot2dof.urdf" />
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="paused" value="$(arg paused)"/>
    </include>    
   
    <node pkg="gazebo_ros" name="spawn_urdf" type="spawn_model" args="-param robot_description -urdf -model robot2dof" />
   
    <!-- -->

</launch>
```

We see a robot, but it is essentially nonfunctional gazebo interface. It doesn't do anything, and is missing lots of key information that ROS would need to use this robot. Whe we started visualization with *urdf_tutorial* we had been using *joint_state_publisher* to specify the pose of each joint. However, the robot itself should provide that information in the real world or in gazebo. Yet without specifying that, Gazebo doesn't know to publish that information. To achieve this and for the robot to be interactive with ROS, we need to specify *Plugins* and *Transmissions*.

#### Gazebo plugin
Gazebo plugin is needed for proper interaction between ROS and Gazebo. We need to dynamically link to the ROS library by adding `<gazebo>` tag before the closing `</robot>` tag:

```xml
    <gazebo>
        <plugin name="control" filename="libgazebo_ros_control.so"/>
    </gazebo> 
```

#### Transmission

For every joint we need to specify a transmission. Transmission declares how the joint will be driven or controlled. Before the closing `</robot>` tag add the following code:

```xml
    <!-- PRENOSI -->
    <transmission name="trans1">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="prvi_sklep">
            <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
        </joint>
        <actuator name="motor1">
            <mechanicalReduction>1</mechanicalReduction>
        </actuator>
    </transmission>

    <transmission name="trans2">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="drugi_sklep">
            <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
        </joint>
        <actuator name="motor2">
            <mechanicalReduction>1</mechanicalReduction>
        </actuator>
    </transmission>
```
