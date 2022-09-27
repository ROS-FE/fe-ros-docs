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

![Image shows rviz and gui for 2 DOF robot](images/Screenshot%20from%202022-06-22%2015-22-10.png)


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

Copy file [panda.zip](./panda.zip) into folder `urdf` and unzip it. 

Run command 

```
roslaunch urdf_tutorial display.launch model:='$(find robot_2dof)/urdf/panda/panda.urdf'
```

In rviz you need to change *Fixed Frame* (in *Global Options*) to *panda_link0*.

### Importing 2 DOF robot in gazebo

First, we will create launch file to load urdf file into gazebo. Go to `launch` folder:

```
touch zagon_gazebo.launch
code zagon_gazebo.launch
```

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

What we see is only empty gazebo world without the robot. This is because the urdf file does not contain everything we need for simulation in gazebo. First, we need to add `<inertial>` tag to each link. Gazebo uses physics engine and needs information about ineratial properties of the bodies, including links ([documentation](https://wiki.ros.org/urdf/XML/link)) of the robots. If `<inertial>` property is not added to the link or mass is zero, Gazebo will ignore the link. It is also quite important for stable physical simulation that inertial parameters are quite accurate or at least have reasonable values.


Below is urdf file with added inertial parameters for each link.

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
        <plugin name="control" filename="libgazebo_ros_control.so">
            <robotNamespace>/MOJROBOT</robotNamespace>
        </plugin>
    </gazebo> 
```



#### Transmission

For every joint we need to specify a transmission. Transmission declares how the joint will be driven or controlled. There are three types of transmissions ([documentation](http://wiki.ros.org/urdf/XML/Transmission
)):
- positional - PositionJointInterface,
- velocity - VelocityJointInterface,
- joint torque - EffortJointInterface.


Before the closing `</robot>` tag add the following code:

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

### Controlling the 2DOF robot

Now that we linked Gazebo and ROS we want to control the robot. We have to setup the controllers.

First we need to add `joint_state_publisher` gazebo plugin in `urdf` file:

```xml
    <gazebo>
        <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
            <jointName>prvi_sklep, drugi_sklep</jointName>
        </plugin>
    </gazebo> 
```

We need to create `yaml` files with specifications for controllers. `yaml` will be saved in `config` folder. Lets move into our `src` folder of our workspace. 

We will create 1 folder:
- folder `config` for storing  `yaml` files.

```
cd robot_2dof
mkdir config
cd config
```

Create `yaml`  file and open it in editor.

```
touch controller.yaml
code controller.yaml
```

Add the following text:

```yaml
MOJROBOT:
  # publish all joint states:
  joint_state_controller:
    type: joint_state_controller/JointStateController
    publish_rate: 10

MOJROBOT/gazebo_ros_control/pid_gains:
  prvi_sklep: {p: 1000, i: 0,  d: 100}
  drugi_sklep: {p: 1000, i: 0,  d: 100}
```

In `launch` file we need to add the following lines:

```xml
    <rosparam file="$(find robot_2dof)/config/controller.yaml" command="load"/>

    <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false" output="screen" ns="/MOJROBOT" 
        args="joint_state_controller"/>

    <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" respawn="false" output="screen">
        <remap from="/joint_states" to="/MOJROBOT/joint_states" />
    </node>
```

Run the launc file `roslaunch robot_2dof zagon_gazebo.launch`.

Now we can observe the joint angles. Type into terminal window command `rostopic list`, we get the list of topics

```
/MOJROBOT/gazebo_ros_control/pid_gains/drugi_sklep/parameter_descriptions
/MOJROBOT/gazebo_ros_control/pid_gains/drugi_sklep/parameter_updates
/MOJROBOT/gazebo_ros_control/pid_gains/prvi_sklep/parameter_descriptions
/MOJROBOT/gazebo_ros_control/pid_gains/prvi_sklep/parameter_updates
/MOJROBOT/joint_states
/clock
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/set_link_state
/gazebo/set_model_state
/joint_states
/rosout
/rosout_agg
/tf
/tf_static
```

Run command `rostopic echo /MOJROBOT/joint_states` and you can now track joint angles.

However, we want not just observe joint angles, but also control the robot. We need to add the controller for the individual joints. We will do that by adding joint controller `joint_controller`. We will first add it to `launch` file to the `controller_spawner`:

```xml
    <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false" output="screen" ns="/MOJROBOT" 
        args="joint_state_controller
        joint_controller"/> <!--Add this new line-->
```

Next we need to add it to the `yaml` file:

```yaml
MOJROBOT:
  # publish all joint states:
  joint_state_controller:
    type: joint_state_controller/JointStateController
    publish_rate: 10
  joint_controller: #Add this new line
    type: position_controllers/JointTrajectoryController #Add this new line
    joints: #Add this new line
    - prvi_sklep #Add this new line
    - drugi_sklep #Add this new line
```

Now when we run the launch file, we get new rostopics:

```
/MOJROBOT/joint_controller/command
/MOJROBOT/joint_controller/follow_joint_trajectory/cancel
/MOJROBOT/joint_controller/follow_joint_trajectory/feedback
/MOJROBOT/joint_controller/follow_joint_trajectory/goal
/MOJROBOT/joint_controller/follow_joint_trajectory/result
/MOJROBOT/joint_controller/follow_joint_trajectory/status
/MOJROBOT/joint_controller/state
```

If we type command `rostopic info /MOJROBOT/joint_controller/command` we can see which type of message we need to publish to control the robot joints:

```
Type: trajectory_msgs/JointTrajectory

Publishers: None

Subscribers: 
 * /gazebo (http://lr-omen:35047/)
```

To move the robot we need to publish the following message:

```
rostopic pub /MOJROBOT/joint_controller/command trajectory_msgs/JointTrajectory '{joint_names:["prvi_sklep","drugi_sklep"],points:[{positions:[-0.5, 0.2], time_from_start:[1,0]}]}'
```

The full files content of the files until this point is [linked](#snapshot-1) bellow.

### Controlling the 2DOF robot using MoveIt

The aim of this section is to use MoveIt package to send the commands to the Gazebo simulation of the robot. Full documentation for MoveIt is http://docs.ros.org/en/melodic/api/moveit_tutorials/html/index.html.
We will use MoveIt Setup Assistant to create MoveIt configuration for our 2DOF robot. We will closely follow the [tutorial](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html) published on MoveIt page. Here we will give really short overview.


[//]: # (Comment)```
[//]: # (Comment)sudo apt-get install ros-melodic-moveit
[//]: # (Comment)sudo apt-get install ros-melodic-moveit-setup-assistant
[//]: # (Comment)cd src/
[//]: # (Comment)git clone -b melodic-devel https://github.com/ros-planning/moveit_tutorials.git
[//]: # (Comment)rosdep install -y --from-paths . --ignore-src --rosdistro melodic
[//]: # (Comment)cd ..
[//]: # (Comment)catkin_make
[//]: # (Comment)```


In terminal run command `roslaunch moveit_setup_assistant setup_assistant.launch`. 

1. In *Start* tab select *Create New MoveIt Configuration Package* and browse for `robot2dof.urdf` file. Load the file.
  ![MoveIt Setup Assistant](images/Screenshot%20from%202022-07-08%2010-54-30.png) 
  ![MoveIt Setup Assistant](images/Screenshot%20from%202022-07-08%2010-55-15.png)  

2. *Self-Collisins* tab.  Click on the *Generate Collision Matrix* button
  ![*Self-Collisins* tab](images/Screenshot%20from%202022-07-08%2010-57-05.png) 

3. *Planning groups* tab. Clock on *Click on Add Group*. You will be presented by new window.
   1. Enter *Group Name* as **arm**.
   2. Choose *kdl_kinematics_plugin/KDLKinematicsPlugin* as the kinematics solver. 
   3. Click on the *Add Joints* button. You will bepresented by new window. Choose the appropriate joints. See figure bellow. Click *Save*.
    ![*Planning groups* tab](images/Screenshot%20from%202022-07-08%2011-00-52.png)

   4. Next figure shows most basic definition for the 2DOF robot.
    ![*Planning groups* tab](images/Screenshot%20from%202022-07-08%2011-01-52.png)

4. *End Effectors* tab. Click *Add End Effector*. You will be presented by new window.
   1. Choose **TCP** as the *End Effector Name*.
   2. Choose **TCP_segment** as the *Parent Link* for this end-effector. Click *Save*.
    ![*End Effectors* tab](images/Screenshot%20from%202022-07-08%2011-13-03.png)
    ![*End Effectors* tab](images/Screenshot%20from%202022-07-08%2011-15-27.png)

5. *ROS Control* tab. Click on *Add Controller*. Add **MOJROBOT/arm_controller* for *Controller Name* and choose *controller type*. See figure bellow. Click *Save* . You might get error *Invalid ROS controller name*. If you get the error close the error dialog. Click the button *Auto Add FollowJointsTrajectory Controllers For Each Planning Group*. We will add the correct controller manually later. 
    ![*Add Controller*](images/Screenshot%20from%202022-07-08%2011-19-01.png)
    ![*Add Controller*](images/Screenshot%20from%202022-07-08%2011-21-58.png) 


6. *Author Information* tab. Enter your name and email address.
7. *Configuration Files* tab. Create folder `robot_2dof_moveit_config` in `src` folder of your workspace. Browse for that folder and click *Generate Package* button. The Setup Assistant will now generate and write a set of launch and config files.
    ![*Configuration Files*](images/Screenshot%20from%202022-07-08%2011-26-35.png)

Now that we have created the MoveIt configuration files, we need to edit them so that they can be used with Gazebo. The auto-generated launch and config files are not ready for use in Gazebo. If you run command in terminal `roslaunch robot_2dof_moveit_config demo_gazebo.launch` you will se that in rviz you can plan movement, but when you want to execurte them the robot in Gazebo does not move. MoveIt generates the file `ros_controllers.yaml` which is missing definition for `controller_list`. We also need to add the namespace for our robot.
The required steps are:
1. File `ros_controllers.yaml`. Add definition for namespace and controller list.
   
```yaml
# Publish all joint states
# Creates the /joint_states topic necessary in ROS
MOJROBOT:
  joint_state_controller:
    type: joint_state_controller/JointStateController
    publish_rate: 50
  arm_controller:
    type: position_controllers/JointTrajectoryController
    joints:
      - prvi_sklep
      - drugi_sklep
    gains:
      prvi_sklep:
        p: 100
        d: 1
        i: 1
        i_clamp: 1
      drugi_sklep:
        p: 100
        d: 1
        i: 1
        i_clamp: 1

controller_list:
  - name: MOJROBOT/arm_controller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - prvi_sklep
      - drugi_sklep  
```

2. File `ros_controllers.launch`. This launch file loads Gazebo controllers. We need to add additional controllers to `controller_spawner` node:

```xml
  <!-- Load the controllers -->
  <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false"
    output="screen" args="--namespace=/MOJROBOT
    joint_state_controller
    arm_controller
    --timeout 20"/>
```

3. File `demo_gazebo.launch`. Definition for node `joint_state_publisher ` has to be changed. Also for `rosparam` `source_list` you need to add correct namespace. Replace the following lines

    ```xml
    <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" unless="$(arg use_gui)">
        <rosparam param="source_list">[move_group/fake_controller_joint_states]</rosparam>
        <rosparam param="source_list">[joint_states]</rosparam>
    </node>
    <node name="joint_state_publisher" pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" if="$(arg use_gui)">
        <rosparam param="source_list">[move_group/fake_controller_joint_states]</rosparam>
        <rosparam param="source_list">[joint_states]</rosparam>
        </node>
    ```

    with

    ```xml
    <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
        <param name="use_gui" value="$(arg use_gui)"/>
        <rosparam param="source_list">[MOJROBOT/joint_states]</rosparam>
    </node>
    ```

The full files content of the files until this point is [linked](#snapshot-2) bellow.

A relatively short explanation of the above process can be found on
https://medium.com/@tahsincankose/custom-manipulator-simulation-in-gazebo-and-motion-planning-with-moveit-c017eef1ea90

Now when you run command `roslaunch robot_2dof_moveit_config demo_gazebo.launch` you can move robot in Gazebo.

![Moving the robot in Gazebo with MoveIt](images/Screenshot%20from%202022-07-08%2012-38-45.png)


### Sensors

We can add various sensors to the robot in `urdf` file. Typically we need to create a link for the sensor and a fixed joint to add the link to the link of the robot. Finally we add `gazebo` and `sensor` tag with appropriate `plugin` and parameters for the sensor.

#### Camera sensor

First we add a link and a joint for the camera sensor in the `urdf` file for the robot.

```xml
    <link name="camera_link">
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry>
                <box size="0.05 0.05 0.05"/>
            </geometry>
            <material name="Black"/>
        </visual>
        <inertial>
            <mass value="1e-5" />
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6" />
        </inertial>
    </link>   

    <joint name="camera_joint" type="fixed">
        <origin xyz="0.0 0.0 0.06" rpy="0 0 0"/>
        <parent link="TCP_segment"/>
        <child link="camera_link"/>
    </joint>
```

Next we add the plugin and input camera parameters.


```xml
    <!-- camera -->
    <gazebo reference="camera_link">
    <material>Gazebo/Black</material>
        <sensor type="camera" name="camera1">
            <update_rate>30.0</update_rate>
            <camera name="head">
                <horizontal_fov>1.3962634</horizontal_fov>
                <image>
                    <width>800</width>
                    <height>800</height>
                    <format>R8G8B8</format>
                </image>
                <clip>
                    <near>0.02</near>
                    <far>300</far>
                </clip>
                <noise>
                    <type>gaussian</type>
                    <mean>0.0</mean>
                    <stddev>0.007</stddev>
                </noise>
            </camera>
            <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
                <alwaysOn>true</alwaysOn>
                <updateRate>0.0</updateRate>
                <cameraName>robot_2dof/camera1</cameraName>
                <imageTopicName>image_raw</imageTopicName>
                <cameraInfoTopicName>camera_info</cameraInfoTopicName>
                <frameName>camera_link</frameName>
                <hackBaseline>0.07</hackBaseline>
                <distortionK1>0.0</distortionK1>
                <distortionK2>0.0</distortionK2>
                <distortionK3>0.0</distortionK3>
                <distortionT1>0.0</distortionT1>
                <distortionT2>0.0</distortionT2>
            </plugin>
        </sensor>
    </gazebo> 
```

#### Laser sensor

First we add a link and a joint for the laser sensor in the `urdf` file for the robot.


```xml
    <!-- Sick Laser -->
    <link name="sick_link">
        <collision>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry>
                <box size="0.1 0.1 0.1"/>
            </geometry>
        </collision>

        <visual>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry>
                <!--<mesh filename="package://rrbot_description/meshes/hokuyo.dae"/>-->
                <box size="0.1 0.1 0.1"/>
            </geometry>
        </visual>

        <inertial>
            <mass value="1e-5" />
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6" />
        </inertial>
    </link>

    <joint name="sick_joint" type="fixed">
        <axis xyz="0 1 0" />
        <origin xyz="0 0.5 0.2" rpy="0 0 0"/>
        <parent link="world"/>
        <!--<parent link="TCP_segment"/>-->
        <child link="sick_link"/>
    </joint>    

```

Next we add the plugin and input sensor parameters.


```xml
    <!-- sick -->
    <gazebo reference="sick_link">
        <sensor type="ray" name="head_sick_sensor">
            <pose>0 0 0 0 0 0</pose>
            <visualize>true</visualize>
            <update_rate>40</update_rate>
            <ray>
                <scan>
                    <horizontal>
                        <samples>720</samples>
                        <resolution>1</resolution>
                        <min_angle>-1.570796</min_angle>
                        <max_angle>1.570796</max_angle>
                    </horizontal>
                </scan>
                <range>
                    <min>0.10</min>
                    <max>30.0</max>
                    <resolution>0.01</resolution>
                </range>
                <noise>
                    <type>gaussian</type>
                    <!-- Noise parameters based on published spec for Hokuyo laser
                    achieving "+-30mm" accuracy at range < 10m.  A mean of 0.0m and
                    stddev of 0.01m will put 99.7% of samples within 0.03m of the true
                    reading. -->
                    <mean>0.0</mean>
                    <stddev>0.01</stddev>
                </noise>
            </ray>
            <plugin name="gazebo_ros_head_sick_controller" filename="libgazebo_ros_laser.so">
                <topicName>/robot_2dof/laser/scan</topicName>
                <frameName>sick_link</frameName>
            </plugin>
        </sensor>
    </gazebo>
```

### Warehouse environment

We will add the model for warehouse environment. 


Copy file [warehouse.zip](./warehouse.zip) into folder `robot_2dof_moveit_config` and unzip it. You must unzip it directly into folder `robot_2dof_moveit_config` so that the folder contains new folders `worlds`, `meshes` and `models`.

In `gazebo.launch` change the line

```xml
    <arg name="world_name" default="worlds/empty.world"/>
```

with

```xml
    <arg name="world_name" value="$(find robot_2dof_moveit_config)/worlds/warehouse_2.world"/>
```   

Before the line
```xml
  <!-- startup simulated world -->
``` 

Add the following lines

```xml
  <!-- warehouse simulation environment -->
  <env name="GAZEBO_MODEL_PATH" value="${GAZEBO_MODEL_PATH}:$(find robot_2dof_moveit_config)/models"/>
  <env name="GAZEBO_RESOURCE_PATH" value="${GAZEBO_RESOURCE_PATH}:$(find robot_2dof_moveit_config)/models"/> 
```

File `warehouse_2.world` contains description of the warehouse environment.


![Warehouse environment rviz](images/Screenshot%20from%202022-07-27%2013-18-05.png)

![Warehouse environment gazebo](images/Screenshot%20from%202022-07-27%2013-18-13.png)

### Franka Panda robot visualization

```
roslaunch franka_gazebo panda.launch x:=-0.5 \
    controller:=position_joint_trajectory_controller \
    rviz:=true
```

```
rostopic info /position_joint_trajectory_controller/command
```

```
rostopic pub /position_joint_trajectory_controller/command trajectory_msgs/JointTrajectory '{joint_names:["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"], points:[{positions:[1.0001325654178492286, -0.7856323608954296, -2.3382642150515665e-05, -2.355961433795475, -8.569228689303543e-06, 1.571747302164006, 0.785388329873661],time_from_start:[1,0]}]}'
```

```
sudo apt-get install ros-melodic-moveit-visual-tools
```

## Code snapshots

### Snapshot 1 

#### File `robot2dof.urdf`

```xml
<?xml version="1.0"?>
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
            <origin xyz="0.12 0.0 0.21" rpy="0.0 0.0 0.0"/>
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
            <origin xyz="-0.08 0.0 0.21" rpy="0.0 0.0 0.0"/>
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

    <gazebo>
        <plugin name="control" filename="libgazebo_ros_control.so">
            <robotNamespace>/MOJROBOT</robotNamespace>
        </plugin>
    </gazebo> 

    <gazebo>
        <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
            <jointName>prvi_sklep, drugi_sklep</jointName>
        </plugin>
    </gazebo>      

</robot>
```

#### File `zagon_gazebo.launch`


```xml
<?xml version="1.0"?>

<launch>

    <arg name="paused" default="true"/>

    <param name="robot_description" textfile="$(find robot_2dof)/urdf/robot2dof.urdf" />
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="paused" value="$(arg paused)"/>
    </include>    
   
    <node pkg="gazebo_ros" name="spawn_urdf" type="spawn_model" args="-param robot_description -urdf -model robot2dof" />
   
    <rosparam file="$(find robot_2dof)/config/controller.yaml" command="load"/>

    <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false" output="screen" ns="/MOJROBOT" 
        args="joint_state_controller
        joint_controller"/> <!--##NOVO-->

    <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" respawn="false" output="screen">
        <remap from="/joint_states" to="/MOJROBOT/joint_states" />
    </node>

    <!-- -->

</launch>
```

#### File `controller.yaml`

```yaml
MOJROBOT:
  # publish all joint states:
  joint_state_controller:
    type: joint_state_controller/JointStateController
    publish_rate: 10
  joint_controller: ##NOVO
    type: position_controllers/JointTrajectoryController ##NOVO
    joints: ##NOVO
    - prvi_sklep ##NOVO
    - drugi_sklep ##NOVO

MOJROBOT/gazebo_ros_control/pid_gains:
  prvi_sklep: {p: 1000, i: 0,  d: 100}
  drugi_sklep: {p: 1000, i: 0,  d: 100}

```

### Snapshot 2

#### File `ros_controllers.yaml`
```yaml
# Simulation settings for using moveit_sim_controllers
moveit_sim_hw_interface:
  joint_model_group: arm
  joint_model_group_pose: home
# Settings for ros_control_boilerplate control loop
generic_hw_control_loop:
  loop_hz: 300
  cycle_time_error_threshold: 0.01
# Settings for ros_control hardware interface
hardware_interface:
  joints:
    - prvi_sklep
    - drugi_sklep
  sim_control_mode: 1  # 0: position, 1: velocity
# Publish all joint states
# Creates the /joint_states topic necessary in ROS
MOJROBOT:
  joint_state_controller:
    type: joint_state_controller/JointStateController
    publish_rate: 50
  arm_controller:
    type: position_controllers/JointTrajectoryController
    joints:
      - prvi_sklep
      - drugi_sklep
    gains:
      prvi_sklep:
        p: 100
        d: 1
        i: 1
        i_clamp: 1
      drugi_sklep:
        p: 100
        d: 1
        i: 1
        i_clamp: 1

controller_list:
  - name: MOJROBOT/arm_controller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - prvi_sklep
      - drugi_sklep    
```

#### File `ros_controllers.launch`

```xml
<?xml version="1.0"?>
<launch>

  <!-- Load joint controller configurations from YAML file to parameter server -->
  <rosparam file="$(find robot_2dof_moveit_config)/config/ros_controllers.yaml" command="load"/>

  <!-- Load the controllers -->
  <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false"
    output="screen" args="--namespace=/MOJROBOT
    joint_state_controller
    arm_controller
    --timeout 20"/>

</launch>
```


#### File `demo_gazebo.launch`

```xml
<launch>

  <!-- By default, we do not start a database (it can be large) -->
  <arg name="db" default="false" />
  <!-- Allow user to specify database location -->
  <arg name="db_path" default="$(find robot_2dof_moveit_config)/default_warehouse_mongo_db" />

  <!-- By default, we are not in debug mode -->
  <arg name="debug" default="false" />

  <!-- By default, we won't load or override the robot_description -->
  <arg name="load_robot_description" default="false"/>

  <!--
  By default, hide joint_state_publisher's GUI

  MoveIt!'s "demo" mode replaces the real robot driver with the joint_state_publisher.
  The latter one maintains and publishes the current joint configuration of the simulated robot.
  It also provides a GUI to move the simulated robot around "manually".
  This corresponds to moving around the real robot without the use of MoveIt.
  -->
  <arg name="use_gui" default="false" />

  <!-- Gazebo specific options -->
  <arg name="gazebo_gui" default="true"/>
  <arg name="paused" default="false"/>
  <!-- By default, use the urdf location provided from the package -->
  <arg name="urdf_path" default="$(find robot_2dof)/urdf/robot2dof.urdf"/>

  <!-- launch the gazebo simulator and spawn the robot -->
  <include file="$(find robot_2dof_moveit_config)/launch/gazebo.launch" >
    <arg name="paused" value="$(arg paused)"/>
    <arg name="gazebo_gui" value="$(arg gazebo_gui)"/>
    <arg name="urdf_path" value="$(arg urdf_path)"/>
  </include>

  <!-- If needed, broadcast static tf for robot root -->
  

  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
    <param name="use_gui" value="$(arg use_gui)"/>
    <rosparam param="source_list">[MOJROBOT/joint_states]</rosparam>
  </node>

  <!-- We do not have a robot connected, so publish fake joint states 
  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" unless="$(arg use_gui)">
    <rosparam param="source_list">[move_group/fake_controller_joint_states]</rosparam>
    <rosparam param="source_list">[MOJROBOT/joint_states]</rosparam>
  </node>
  <node name="joint_state_publisher" pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" if="$(arg use_gui)">
    <rosparam param="source_list">[move_group/fake_controller_joint_states]</rosparam>
    <rosparam param="source_list">[MOJROBOT/joint_states]</rosparam>
  </node>-->

  <!-- Given the published joint states, publish tf for the robot links -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" respawn="true" output="screen" />

  <!-- Run the main MoveIt! executable without trajectory execution (we do not have controllers configured by default) -->
  <include file="$(find robot_2dof_moveit_config)/launch/move_group.launch">
    <arg name="allow_trajectory_execution" value="true"/>
    <arg name="fake_execution" value="false"/>
    <arg name="info" value="true"/>
    <arg name="debug" value="$(arg debug)"/>
    <arg name="load_robot_description" value="$(arg load_robot_description)"/>
  </include>

  <!-- Run Rviz and load the default config to see the state of the move_group node -->
  <include file="$(find robot_2dof_moveit_config)/launch/moveit_rviz.launch">
    <arg name="rviz_config" value="$(find robot_2dof_moveit_config)/launch/moveit.rviz"/>
    <arg name="debug" value="$(arg debug)"/>
  </include>

  <!-- If database loading was enabled, start mongodb as well -->
  <include file="$(find robot_2dof_moveit_config)/launch/default_warehouse_db.launch" if="$(arg db)">
    <arg name="moveit_warehouse_database_path" value="$(arg db_path)"/>
  </include>

</launch>

```


https://github.com/wh200720041/warehouse_simulation_toolkit


[robot_2dof_moveit_config.zip](./robot_2dof_moveit_config.zip)