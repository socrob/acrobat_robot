import gym
import rospy
import roslaunch
import time
import numpy as np

from gym import utils, spaces
from gym.spaces.box import Box
#from gym_gazebo.envs import gazebo_env
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

from sensor_msgs.msg import LaserScan

from gym.utils import seeding

from gazebo_env_baseclass import GazeboEnv

from std_msgs.msg import Float32

from sensor_msgs.msg import JointState 

from gazebo_msgs.srv import SetModelConfiguration

import random



class GazeboAcrobatEnv(GazeboEnv):

    def __init__(self):
        # Launch the simulation with the given launchfile name
        GazeboEnv.__init__(self, "acrobat_world.launch",log = "rospy.FATAL")

        # Topic to publish torque
        self.torque_pub = rospy.Publisher('/acrobat/joint1/effort/command', Float32, queue_size=5)

        #Topic to read the state of the joint
        self.joint_state = rospy.Subscriber('/joint_states', JointState)

        # Gazebo Service that resets simulation
        rospy.wait_for_service('/gazebo/reset_simulation')
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

        # Gazebo service to pause
        rospy.wait_for_service('/gazebo/pause_physics')
        p = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.pausa = p

        # Gazebo service to unpause
        rospy.wait_for_service('/gazebo/unpause_physics')
        up = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.fora_pausa = up

        rospy.wait_for_service('/gazebo/set_model_configuration')
        angus = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
        self.set_joint1_angle_service = angus

        self.action_space = Box(low=-10, high=10, shape=(1,))
        self.reward_range = (-np.inf, np.inf)

        self._seed()

    def setJointAngle(self, angle):
        '''
        Uses /gazebo/set_model_configuration to set a position in the acrobat without using any controllers
        '''
        # call service /gazebo/set_model_configuration
        return self.set_joint1_angle_service('acrobat', 'robot_description', ['joint1'], [angle]) 

    @property
    def get_angle(self):
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/joint_states', JointState, timeout=5)
            except:
                pass
        rospy.loginfo("I heard {}".format(data.position))
        return data

    @property
    def get_state(self):
        data = self.get_angle
        angle = data.position
        velocity = data.velocity
        l = []
        l.append(np.cos(angle[0]))
        l.append(np.sin(angle[0]))
        l.append(velocity[0])
        return np.asarray(l)


    def publish_torque(self, torque):
        self.torque_pub.publish(data=torque)
        rospy.sleep(0.1)
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/joint_states', JointState, timeout=5)
            except:
                pass
        rospy.loginfo("I heard {}".format(data.effort))
        rospy.loginfo("Number of connections {}".format(self.torque_pub.get_num_connections()))


    # Defines a seed
    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    # Makes a step
    def step(self, action):

        self.unpause
        self.publish_torque(action[0])
        state = self.get_state
        self.pause
        reward = self.reward(state,action)
        done = False

        return state, reward, done


    def reward(self,state,action):
        angle = np.arcsin(state[0])
        #print("Current angle is {}".format(angle))
        return -1 * (angle**2 + 0.1*state[2]**2 + 0.001*action[0]**2)

    # Resets
    @property
    def reset(self):

        # Resets the state of the environment and returns an initial observation.
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print ("/gazebo/reset_simulation service call failed")

        # Unpause simulation to make observation
        self.unpause
        
        # Reset Topic
        self.torque_pub.publish(data=0)
        self.setJointAngle(random.uniform(-3.1415, 3.1415))
        
        # Read joint states
        state = self.get_state

        #Pause for next step
        self.pause

        return state

    # Pauses
    @property
    def pause(self):
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pausa()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

    # Unpauses
    @property
    def unpause(self):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.fora_pausa()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

    


