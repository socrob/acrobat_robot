Gazebo ROS Acrobat balancing robot
===

* Author: Oscar Lima <olima@isr.tecnico.ulisboa.pt>
* Co-Author: Daniel Marta <daniellsmarta@gmail.com>
* License: GNU General Public License, version 3 (GPL-3.0)

Test your learning algorithms with this balancing 1 link 1 joint robot.

It can also be used as an example on how to create a simple 1 joint robot
with Gazebo and ros control.

It also contains an example of how to create a simple plugin to exert a force on the robot joint.

![alt gazebo_acrobat_robot](https://github.com/socrob/acrobat_robot/blob/kinetic/gazebo_acrobat_robot.png "Gazebo acrobat 1 joint robot")

Quick Start
===

Run acrobat robot Gazebo simulation:

    roslaunch acrobat_gazebo acrobat_world.launch

Publish desired joint effort on topic:

    rostopic pub /acrobat/joint1/effort/command std_msgs/Float32 "data: 5.0"

Access acrobat joint1 angle:

    rostopic echo /acrobat/joint1/angle

Access acrobat reward function based on euclidean distance (NEW feature!):

    rostopic echo /tf_euclidean_distance_node/euclidean_distance

Credits: Partially based on the rrbot Gazebo tutorial, located under
===

    https://github.com/ros-simulation/gazebo_ros_demos
