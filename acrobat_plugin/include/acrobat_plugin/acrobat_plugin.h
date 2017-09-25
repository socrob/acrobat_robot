//from rrbot
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>

// Gazebo, from gazebo_ros_control_plugin
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>

#include <gazebo_plugins/gazebo_ros_utils.h>

//#include <gazebo/common/common.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

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

    // A node use for ROS transport
    std::unique_ptr<ros::NodeHandle> rosNode;
    
    // A ROS subscriber
    ros::Subscriber rosSub;
    
    // A ROS callbackqueue that helps process messages
    ros::CallbackQueue rosQueue;
    
    // A thread the keeps running the rosQueue
    std::thread rosQueueThread;
    
    double effort_ = 5.0;
    
  protected:
    
    // Pointer to the model
    gazebo::physics::ModelPtr parent_model_;
    sdf::ElementPtr sdf_;
    
    // Pointer to the update event connection
    gazebo::event::ConnectionPtr update_connection_;
    
    std::vector<gazebo::physics::JointPtr> sim_joints_;

};
