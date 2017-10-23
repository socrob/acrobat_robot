import gym
import rospy
import roslaunch
import time
import numpy as np

from gym import utils, spaces
#from gym_gazebo.envs import gazebo_env
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

from sensor_msgs.msg import LaserScan

from gym.utils import seeding

from gazebo_env_baseclass import GazeboEnv

from std_msgs.msg import Float32

from sensor_msgs.msg import JointState 


class GazeboAcrobatEnv(GazeboEnv):

    def __init__(self):
        # Launch the simulation with the given launchfile name
        GazeboEnv.__init__(self, "acrobat_world.launch")

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

        self.action_space = spaces.Discrete(3) #F,L,R
        self.reward_range = (-np.inf, np.inf)

        self._seed()

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
        return np.cos(angle),np.sin(angle),velocity


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


    def discretize_observation(self,data,new_ranges):
        discretized_ranges = []
        min_range = 0.2
        done = False
        mod = len(data.ranges)/new_ranges
        for i, item in enumerate(data.ranges):
            if (i%mod==0):
                if data.ranges[i] == float ('Inf') or np.isinf(data.ranges[i]):
                    discretized_ranges.append(6)
                elif np.isnan(data.ranges[i]):
                    discretized_ranges.append(0)
                else:
                    discretized_ranges.append(int(data.ranges[i]))
            if (min_range > data.ranges[i] > 0):
                done = True
        return discretized_ranges,done

    # Defines a seed
    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    # Makes a step
    def step(self, action):

        self.unpause
        self.publish_torque(action)
        state = self.get_state
        self.pause

        #state,done = self.discretize_observation(data,5)
        #state = self.get_state
        reward = 100
        done = True
        #if not done:
        #    if action == 0:
        #        reward = 5
        #    else:
        ##        reward = 1
        #else:
        #    reward = -200

        return state


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


