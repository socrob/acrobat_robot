/*
 * Copyright [2017] <Instituto Superior Tecnico>
 *
 * Author: Oscar Lima (olima@isr.tecnico.ulisboa.pt)
 *
 * Gazebo plugin to exert a force on the acrobat single joint based on input topic.
 *
 * Input: This node subcribes to: /gazebo_client/acrobat_joint_effort_command (std_msgs::Float32)
 *        Will exert that force on the joint (acrobat joint1, which is the only joint that the robot has)
 *
 * Output: Will communicate with the Gazebo robot and will apply that force to your joint
 *
 */

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <thread>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>

class AcrobatJointPlugin : public gazebo::ModelPlugin
{
    // No constructor, notice that this class inherits from gazebo::ModelPlugin

  public:
    // Destructor method
    virtual ~AcrobatJointPlugin();

    // Overloaded Gazebo entry point
    virtual void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf);

    // Called by the world update start event
    void Update();

    void OnRosMsg(const std_msgs::Float32ConstPtr &_msg);

    void QueueThread();

  private:

    // a pointer to the world
    gazebo::physics::WorldPtr world_;

    // A node use for ROS transport
    std::unique_ptr<ros::NodeHandle> rosNode;

    // A ROS subscriber
    ros::Subscriber rosSub;

    // A ROS callbackqueue that helps process messages
    ros::CallbackQueue rosQueue;

    // A thread the keeps running the rosQueue
    std::thread rosQueueThread;

    // for publishing
    ros::NodeHandle* nh_;
    ros::Publisher acrobat_joint_angle_publisher_;

    double effort_ = 5.0;

    // to publish joint states
    sensor_msgs::JointState joint_state_msg_;

  protected:

    // Pointer to the model
    gazebo::physics::ModelPtr parent_model_;
    sdf::ElementPtr sdf_;

    // Pointer to the update event connection
    gazebo::event::ConnectionPtr update_connection_;

    std::vector<gazebo::physics::JointPtr> sim_joints_;

};
