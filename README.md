Gazebo ROS Acrobat balancing robot
===

* Author: Oscar Lima <olima@isr.tecnico.ulisboa.pt>
* License: GNU General Public License, version 3 (GPL-3.0)

Test your learning algorithms with this balancing 1 link 1 joint robot.

It can also be used as an example on how to create the simplest robot posible
with Gazebo and ros control.

It also contains an example of how to create a simple plugin to exert a force on a robot joint.

Quick Start
===

Rviz:

    roslaunch acrobat_description acrobat_rviz.launch

Gazebo:

    roslaunch acrobat_gazebo acrobat_world.launch

ROS Control:

    roslaunch acrobat_control acrobat_control.launch

Example of Moving Joints:

    rostopic pub /acrobat/joint1_position_controller/command std_msgs/Float64 "data: -0.4"

Credits: Partially based on the rrbot Gazebo tutorial, located under
===

    https://github.com/ros-simulation/gazebo_ros_demos
