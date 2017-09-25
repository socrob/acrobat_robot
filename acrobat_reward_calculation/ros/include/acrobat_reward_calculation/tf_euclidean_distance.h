/* 
 * Copyright [2017] <Instituto Superior Tecnico>  
 * 
 * Author: Oscar Lima (olima@isr.tecnico.ulisboa.pt)
 *
 * Publishes the difference between two frames
 * as euclidean distance.
 * 
 */

#ifndef ACROBAT_REWARD_CALCULATION_TF_EUCLIDEAN_DISTANCE_H
#define ACROBAT_REWARD_CALCULATION_TF_EUCLIDEAN_DISTANCE_H

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float32.h>
#include <math.h>

class TFEuclideanDistance
{
    public:
        TFEuclideanDistance();
        ~TFEuclideanDistance();
        
        // get parameters from parameter server
        void getParams();
        
        // perform node operation
        void update();
        
        // the condition to enter the running function
        bool condition_to_process();

    private:
        // ros related variables
        ros::Publisher pub_tf_error_euclidean_;
        
        std::string reference_frame_id_;
        std::string target_frame_id_;
        
        // to store the path in which the path will be logged
        std::string path_log_file_;
        
        // the fixed global frame, usually map frame
        std::string global_frame_id_;
        
        tf::TransformListener tf_listener_;
        
        // to store current time and use can transform, wait for transform and 
        // lookup transform with the same time
        ros::Time now_;
        
        ros::NodeHandle nh_;
};
#endif  // ACROBAT_REWARD_CALCULATION_TF_EUCLIDEAN_DISTANCE_H
