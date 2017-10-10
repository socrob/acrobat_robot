#!/usr/bin/env python

import rospy
import math
# import acrobat_srv.step.srv as torque_to_observation_service

from std_srvs.srv import Empty 
from sensor_msgs.msg import JointState

class AcrobatWrapper(object):
    '''
    This class aims to wrapp the ROS-Gazebo Acrobat in an approriate handler for any RL algorithm to act upon. 
    The agent will act on a proxy of this class.
    '''
    def __init__(self, name='Acrobat'):
        # member variables
        self.name = name
        self.step = 0
        # flag that indicates that a joint state msg has being received
        self.joint_state_received = False
        # to store the joint state msg
        self.joint_state_msg = None
        # acrobat gazebo services
        # self.torque_to_observation = rospy.ServiceProxy('torque_to_observation', torque_to_observation_service)
        self.reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.pause_physics = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause_physics = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        # subscribe to joint states topic
        rospy.Subscriber("/joint_states", JointState, self.jointStatesCallback, queue_size=1)
        # fetch from param server the loop rate, to control the frequency at which this node will run
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))
        # inform the user that initialization is done
        rospy.loginfo(self.name + " wrapper node initialized...")


    def jointStatesCallback(self, msg):
        '''
        This function will get executed every time you get a joint state msg
        This will depend upon the frequency that you have selected for gazebo to work, check:
        acrobat_description/urdf acrobat_ros_control_gazebo.urdf.xacro line that says : <updateRate>30.0</updateRate>
        So by default you get 30 joint state msgs per second
        '''
        # make bkp of received data on member variable
        self.joint_state_msg = msg
        # raise flag indicating that a joint msg has been received
        self.joint_state_received = True

    def step(self, torque):
        '''
        Takes the torque on the joint as input and outputs an RL tuple (observation_after, reward, done_flag)
        '''
        self.unpause_physics() 
        try:
            response = self.torque_to_observation(torque)
        except rospy.ServiceException as exc:
            print("At step {} Service did not process request: {} ".format(self.step, str(exc)))
        # The pause probably has to be done better.
        self.pause_physics()
        self.step += 1
        rospy.loginfo("Performing a step!")
        return response[0], response[1], response[2]


    def reset(self):
        '''
        This method should reset the gazebo environment and return an initial random angle (under initial constraints)
        '''
        self.step = 0
        try:
            self.reset_world()
        except rospy.ServiceException as exc:
            print("Service did not process request: {} ".format(str(exc)))
        self.reset_world()
        rospy.loginfo("Resetting the environment")
        # Probably we need to create a service here to set a random angle after resetting the world
        return observation_initial

    
    def render(self):
        '''
        This method should render the simulation in gazebo and call gazebo_client
        '''
        raise NotImplementedError


    def __close__(self):
        '''
        This method should shutdown the openned services properly and free associated memory
        For parallel training this will become quite handy to prevent memory leaks
        '''
        self.torque_to_observation.shutdown('Closing torque_to_observation service')
        self.reset_world.shutdown('Closing reset_world service')
        self.pause_physics.shutdown('Closing pause_physics service')
        self.unpause_physics.shutdown('Closing unpause_physics service')
        del self.name
        rospy.loginfo(self.name + " shutting down!")

    
    @property
    def get_name(self):
        return self.name


    @property
    def environment_shape(self):
        '''
        Should return a tuple with the dimensions of the observation space
        '''
        return (3,)


    @property
    def action_shape(self):
        '''
        Should return a tuple with the dimensions of the action space
        '''
        return (1,)


    def restrict_angle(self, angle):
        '''
        constrain angle between 2 * pi range (0 - 360 degree)
        input: angle in radians
        output: angle in radians constrained to -pi to pi range
        '''
        # apply desired offset (this makes the upper position to be zero
        restricted_angle = angle + math.pi * 2.0
        # remove 360 degree if angle is greater than 360 degree until its between desied bounds
        while restricted_angle > (math.pi * 2.0):
            restricted_angle -= math.pi * 2.0
        return restricted_angle


    def update_rl_algorithm(self, angle):
        '''
        This is where the magic RL happens ; )
        '''
        # position (acrobat angle in radians, acrobat being up is 0 rad)
        rospy.loginfo("acrobat angular position [rad] : " + str(angle))
        # velocity (acrobat angular joint velocity)
        # this if fixes a problem with velocities sometimes comming with empty msg
        if len(self.joint_state_msg.velocity) == 0:
            acrobat_speed = 0.0
        else:
            acrobat_speed = self.joint_state_msg.velocity[0]
        rospy.loginfo("acrobat angular speed [rad/s] : " + str(acrobat_speed))
        # effort (acrobat torque)
        # this if fixes a problem with effort sometimes comming with empty msg
        if len(self.joint_state_msg.effort) == 0:
            acrobat_effort = 0.0
        else:
            acrobat_effort = self.joint_state_msg.effort[0]
        rospy.loginfo("acrobat effort (torque) [N] : " + str(acrobat_effort))
        # dummy print to separate info between msgs
        rospy.loginfo("-------")
        

    def start_acrobat_wrapper(self):
        '''
        acrobat wrapper main loop function
        '''
        while not rospy.is_shutdown():
            if self.joint_state_received == True:
                # lower flag
                self.joint_state_received = False
                # convert position angle to a value between bounds -pi to pi
                restricted_angle = self.restrict_angle(self.joint_state_msg.position[0])
                # do something with received readings
                self.update_rl_algorithm(restricted_angle)
            self.loop_rate.sleep()


def main():
    # register node in the network
    rospy.init_node('acrobat_wrapper', anonymous=False)
    # create object of this class
    acrobat_wrapper = AcrobatWrapper()
    # call class main loop method
    acrobat_wrapper.start_acrobat_wrapper()
