/* 
 * Copyright [2017] <Instituto Superior Tecnico>  
 * 
 * Author: Oscar Lima (olima@isr.tecnico.ulisboa.pt)
 *
 * Publishes the difference between two frames
 * as euclidean distance.
 * 
 */

#include <cstdio>
#include "tf/transform_listener.h"
#include "ros/ros.h"

#define _USE_MATH_DEFINES

#include <acrobat_reward_calculation/tf_euclidean_distance.h>

TFEuclideanDistance::TFEuclideanDistance(): nh_("~")
{
    pub_tf_error_euclidean_ = 
    nh_.advertise<std_msgs::Float32>("euclidean_distance", 1);
    
    getParams();
}

TFEuclideanDistance::~TFEuclideanDistance()
{
    pub_tf_error_euclidean_.shutdown();
}

void TFEuclideanDistance::getParams()
{
    // get required parameters from parameter server
    nh_.param("reference_frame_id", reference_frame_id_, std::string("reference_frame"));
    nh_.param("target_frame_id", target_frame_id_, std::string("target_frame"));
    
    // inform the user about the parameters that will be used
    ROS_INFO("reference_frame_id : %s", reference_frame_id_.c_str());
    ROS_INFO("target_frame_id : %s", target_frame_id_.c_str());
}

void TFEuclideanDistance::update()
{
    try
    {
        //listen to latest available transform
        tf::StampedTransform transform;
        tf_listener_.lookupTransform(reference_frame_id_.c_str(), target_frame_id_.c_str(), now_, transform);
        
        // example on how to access roll, pitch and yaw
        // double yaw, pitch, roll;
        // transform.getBasis().getRPY(roll, pitch, yaw);
        // tf::Quaternion q = transform.getRotation();
        
        tf::Vector3 v = transform.getOrigin();
        
        // create empty msg
        std_msgs::Float32 euclidean_distance_msg;
        
        // compute euclidean distance
        euclidean_distance_msg.data = sqrt(v.getX() * v.getX() + v.getY() * v.getY() + v.getZ() * v.getZ());
        
        // publish
        pub_tf_error_euclidean_.publish(euclidean_distance_msg);
    }
    catch(tf::TransformException& ex)
    {
        // since we are using previously can transform, this code should not get executed
        ROS_ERROR("Lookup transform failed with the following error msg : %s", ex.what());
    }
}

bool TFEuclideanDistance::condition_to_process()
{
    // the condition to enter the running function
    
    // get the current time from ROS time api
    now_ = ros::Time::now();
    
    // wait 1 second for transform to become available
    tf_listener_.waitForTransform(reference_frame_id_.c_str(), target_frame_id_.c_str(),
                              now_, ros::Duration(1.0));
    
    std::string error = "";
    
    // check if transform is available
    if (tf_listener_.canTransform (target_frame_id_.c_str(), reference_frame_id_.c_str(), now_, &error))
    {
        ROS_DEBUG("transform is available");
        return true;
    }
    else
    {
        ROS_DEBUG("Transform between %s [source] and %s [target] not available", 
                 reference_frame_id_.c_str(), target_frame_id_.c_str());
        ROS_WARN("Can transform failed with the following error msg : %s", error.c_str());
    }
    
    return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_error_calculator");

    ROS_INFO("Node is going to initialize...");

    // create object of the node class
    TFEuclideanDistance tf_error_calculator;

    // setup node frequency
    double node_frequency = 0.0;
    ros::NodeHandle nh("~");
    nh.param("node_frequency", node_frequency, 10.0);
    ROS_INFO("Node will run at : %lf [hz]", node_frequency);
    ros::Rate loop_rate(node_frequency);

    ROS_INFO("Node initialized.");

    while (ros::ok())
    {
        if(tf_error_calculator.condition_to_process())
        {
            // main loop function
            tf_error_calculator.update();
        }

        // sleep to control the node frequency with ROS time api
        loop_rate.sleep();
    }

    return 0;
}
