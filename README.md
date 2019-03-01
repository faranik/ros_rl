# ros_rl
This project create an ROS environment for testing Reinforcement Learning algorithms using the [openai_ros](http://wiki.ros.org/openai_ros) package and [gazebo](http://gazebosim.org/) simulator.

## Dev Environment 
* ROS Kinetic
* Gazebo 7.0.0
* Ubuntu 16.04

For the purpose of usability, all packages and files needed for this project are included in this repository. For more details about these packages or for the exploration of new versions please refer to the links below:
* https://bitbucket.org/theconstructcore/openai_ros
* https://github.com/turtlebot/turtlebot.git

## Add a new gym environment
* Refer to [Building a world](http://gazebosim.org/tutorials?tut=build_world) tutorial to create a world for gazebo environment. Once the new world is created, save it to gym_envs/worlds folder and give it a meaningful name.
* Under gym_envs/launch, create a new launch file. You can use the `main.launch` file as an example / template for your own file or use the example below:
  ```
  <?xml version="1.0" encoding="utf-8"?>
  <launch>
    <arg name="base"      value="$(optenv TURTLEBOT_BASE kobuki)"/> <!-- create, roomba -->
    <arg name="battery"   value="$(optenv TURTLEBOT_BATTERY /proc/acpi/battery/BAT0)"/>  <!-- /proc/acpi/battery/BAT0 -->
    <arg name="paused" value="false"/>
    <arg name="gui" default="true"/>
    <arg name="headless" value="true"/>
    <arg name="verbose" value="true"/>
    <arg name="stacks"    value="$(optenv TURTLEBOT_STACKS hexagons)"/>  <!-- circles, hexagons -->
    <arg name="3d_sensor" value="$(optenv TURTLEBOT_3D_SENSOR kinect)"/>  <!-- kinect, asus_xtion_pro -->

    <include file="$(find gazebo_ros)/launch/empty_world.launch">
      <arg name="use_sim_time" value="true"/>
      <arg name="debug" value="false"/>
      <arg name="gui" value="$(arg gui)" />
      <arg name="world_name" value="$(find gym_envs)/worlds/maze_loop_brick.world"/>
    </include>

    <include file="$(find turtlebot_gazebo)/launch/includes/$(arg base).launch.xml">
      <arg name="base" value="$(arg base)"/>
      <arg name="stacks" value="$(arg stacks)"/>
      <arg name="3d_sensor" value="$(arg 3d_sensor)"/>
    </include>

    <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher">
      <param name="publish_frequency" type="double" value="30.0" />
    </node>
  </launch>
  ```
  Observe how the line 16 specifies the world to be launched. Change it for your own world file. Line 15 specifies if to launch the user interface window of Gazebo (default value is true as you can see at line 6). Don't pay attention for now to lines 19 - 23. I'll come back to them when we will talk about how to add a robot to the environment. Save the launch file with a meaningful for you name.
* Build, source and launch your new gym environment.
  ```
  $ cd ~/catkin_ws
  $ catkin_make
  $ source devel/setup.bash
  $ roslaunch gym_envs <name of your new launch file>
  ```
