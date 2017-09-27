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

#include <acrobat_plugin/acrobat_plugin.h>

AcrobatJointPlugin::~AcrobatJointPlugin()
{
    // Disconnect from gazebo events
    gazebo::event::Events::DisconnectWorldUpdateBegin(update_connection_);
}

void AcrobatJointPlugin::Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf)
{
    ROS_INFO("Loading acrobat joint interface plugin");

    // Save pointers to the model
    parent_model_ = parent;
    sdf_ = sdf;

    // Get the world name.
    world_ = parent_model_->GetWorld();

    // Error message if the model couldn't be found
    if (!parent_model_)
    {
        ROS_ERROR_STREAM_NAMED("loadThread","parent model is NULL");
        return;
    }

    // Check that ROS has been initialized
    if(!ros::isInitialized())
    {
        ROS_FATAL_STREAM_NAMED("acrobat_plugin","A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libacrobat_plugin.so' in the acrobat_description package)");
        return;
    }

    // Listen to the update event. This event is broadcast every simulation iteration.
    update_connection_ =
      gazebo::event::Events::ConnectWorldUpdateBegin
      (boost::bind(&AcrobatJointPlugin::Update, this));

    // get a pointer to control the acrobat joint
    gazebo::physics::JointPtr joint = parent_model_->GetJoint(std::string("joint1"));
    sim_joints_.push_back(joint);

    // set joint force to 0 at startup
    effort_ = 0.0;

    // Create our ROS node. This acts in a similar manner to the Gazebo node
    // with code from http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i6
    AcrobatJointPlugin::rosNode.reset(new ros::NodeHandle("acrobat"));

    // Create a named topic, and subscribe to it
    ros::SubscribeOptions so =
    ros::SubscribeOptions::create<std_msgs::Float32>(
        "joint1/effort/command",
        1,
        boost::bind(&AcrobatJointPlugin::OnRosMsg, this, _1),
        ros::VoidPtr(), &this->rosQueue);

    AcrobatJointPlugin::rosSub = AcrobatJointPlugin::rosNode->subscribe(so);

    // Spin up the queue helper thread.
    AcrobatJointPlugin::rosQueueThread =
    std::thread(std::bind(&AcrobatJointPlugin::QueueThread, this));

    // publisher setup
    this->nh_ = new ros::NodeHandle("");
    this->acrobat_joint_angle_publisher_ = this->nh_->advertise<sensor_msgs::JointState>(std::string("joint_states"), 1);

    // set size of joint msg to 1
    joint_state_msg_.name.resize(1);
    joint_state_msg_.position.resize(1);
    joint_state_msg_.velocity.resize(1);
    joint_state_msg_.effort.resize(1);

    // setting joint name from sdf
    if (!sdf_->HasElement("jointName"))
    {
        ROS_FATAL_NAMED("acrobat gazebo plugin", "missing <jointName>, can't proceed");
        return;
    }
    else
    {
        joint_state_msg_.name[0] = sdf_->GetElement("jointName")->Get<std::string>();
    }
}

// Called by the world update start event
void AcrobatJointPlugin::Update()
{
    /* example on how to interact with the joint:
     *
     *  sim_joints_[0]->SetPosition(0, joint_position_command_[j]);
     *  sim_joints_[0]->SetAngle(0, joint_position_command_[j]);
     *  sim_joints_[0]->GetAngle(0).Radian();
     *  sim_joints_[0]->SetVelocity(0, e_stop_active_ ? 0 : joint_velocity_command_[j]);
     *  sim_joints_[0]->GetVelocity(0);
     *  sim_joints_[0]->SetForce(0, effort_);
     *  sim_joints_[0]->GetForce((unsigned int)(0));
     */

    // set force to the value received by topic
    sim_joints_[0]->SetForce(0, effort_);

    // publish acrobat joint state

    // fill header with current timeoutthis->wrench_msg_.header.frame_id = this->frame_name_;
    joint_state_msg_.header.stamp.sec = (world_->GetSimTime()).sec;
    joint_state_msg_.header.stamp.nsec = (world_->GetSimTime()).nsec;

    // ROS_INFO_STREAM( "angle : " << sim_joints_[0]->GetAngle(0).Radian());
    joint_state_msg_.position[0] = sim_joints_[0]->GetAngle(0).Radian();
    joint_state_msg_.velocity[0] = sim_joints_[0]->GetVelocity(0);
    joint_state_msg_.effort[0] = sim_joints_[0]->GetForce((unsigned int)(0));

    this->acrobat_joint_angle_publisher_.publish(joint_state_msg_);
}

// Handle an incoming message from ROS
void AcrobatJointPlugin::OnRosMsg(const std_msgs::Float32ConstPtr &msg)
{
    effort_ = msg->data;
}

// ROS helper function that processes messages
void AcrobatJointPlugin::QueueThread()
{
    static const double timeout = 0.01;
    while (AcrobatJointPlugin::rosNode->ok())
    {
        AcrobatJointPlugin::rosQueue.callAvailable(ros::WallDuration(timeout));
    }
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(AcrobatJointPlugin);
