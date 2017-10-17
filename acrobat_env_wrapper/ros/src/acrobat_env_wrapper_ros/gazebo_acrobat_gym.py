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
        self.torque_pub = rospy.Publisher('/acrobat/joint1/effort/command', Float32, queue_size=5)
        self.joint_state = rospy.Subscriber('/joint_states', JointState)

        rospy.wait_for_service('/gazebo/reset_simulation')
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

        rospy.wait_for_service('/gazebo/pause_physics')
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)

        rospy.wait_for_service('/gazebo/unpause_physics')
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

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
        #self.vel_pub.publish(vel_cmd)
        self.torque_pub.publish(data=torque)
        rospy.sleep(0.5)
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/joint_states', JointState, timeout=5)
            except:
                pass
        rospy.loginfo("I heard {}".format(data.effort))
        rospy.loginfo("Number of connections {}".format(self.torque_pub.get_num_connections()))



    #def callback(self,data):
    #    rospy.loginfo("I heard {}".format(data.position))

    def listener(self):
        rospy.init_node('hehe')
        self.joint_state
        #rospy.spin()

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

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):

        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        if action == 0: #FORWARD
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.3
            vel_cmd.angular.z = 0.0
            self.vel_pub.publish(vel_cmd)
        elif action == 1: #LEFT
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.05
            vel_cmd.angular.z = 0.3
            self.vel_pub.publish(vel_cmd)
        elif action == 2: #RIGHT
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.05
            vel_cmd.angular.z = -0.3
            self.vel_pub.publish(vel_cmd)

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/scan', LaserScan, timeout=5)
            except:
                pass

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        state,done = self.discretize_observation(data,5)

        if not done:
            if action == 0:
                reward = 5
            else:
                reward = 1
        else:
            reward = -200

        return state, reward, done, {}

    def reset(self):

        # Resets the state of the environment and returns an initial observation.
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            #reset_proxy.call()
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print ("/gazebo/reset_simulation service call failed")

        # Unpause simulation to make observation
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            #resp_pause = pause.call()
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        #read joint states
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/joint_states', JointState, timeout=5)
            except:
                pass

        #rospy.wait_for_service('/gazebo/pause_physics')
        #try:
            #resp_pause = pause.call()
        #    self.pause()
        #except (rospy.ServiceException) as e:
        #    print ("/gazebo/pause_physics service call failed")

        state = self.get_state

        return state

    @property
    def pause(self):
        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

    @property
    def unpause(self):
        try:
            #resp_pause = pause.call()
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")


