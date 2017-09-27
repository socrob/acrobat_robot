import rospy
import acrobat_srv.step.srv as torque_to_observation_service
import std_srvs import Empty 
''' 
This class aims to wrapp the ROS-Gazebo Acrobat in an approriate handler for any RL algorithm to act upon. 
The agent will act on a proxy of this class.
'''
class AcrobatWrapper(object):
    def __init__(self, name='Acrobat'):
        self.name = name
        self.step = 0
        rospy.init_node(self.name)
        self.torque_to_observation = rospy.ServiceProxy('torque_to_observation', torque_to_observation_service)
        self.reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.pause_physics = rospy.ServiceProxy('/gazebo/pause_physics',Empty)
        self.unpause_physics = rospy.ServiceProxy('/gazebo/unpause_physics',Empty)
        rospy.loginfo(self.name+" Wrapper Initialized")

    def step(self,torque):
        ''' Takes the torque on the joint as input and outputs an RL tuple (observation_after, reward, done_flag) '''    
        self.unpause_physics() 
        try:
            response = self.torque_to_observation(torque)
        except rospy.ServiceException as exc:
            print("At step {} Service did not process request: {} ".format(self.step,str(exc)))
        # The pause problably has to be done better.
        self.pause_physics()
        self.step += 1
        rospy.loginfo("Performing a step!")
        return response[0], response[1], response[2]

    def reset(self):
        ''' This method should reset the gazebo environment and return an initial random angle (under initial constraints) '''
        self.step = 0
        try:
            self.reset_world()
        except rospy.ServiceException as exc:
            print("Service did not process request: {} ".format(str(exc)))
        self.reset_world()
        rospy.loginfo("Resetting the environment")
        #Problaly we need to create a service here to set a random angle after resetting the world
        return observation_initial
    
    def render(self):
        ''' This method should render the simulation in gazebo and call gazebo_client '''
        raise NotImplementedError


    def __close__(self):
        ''' This method should shutdown the openned services properly and free associated memory 
            For parallel training this will become quite handy to prevent memory leaks'''
        self.torque_to_observation.shutdown('Closing torque_to_observation service')
        self.reset_world.shutdown('Closing reset_world service')
        self.pause_physics.shutdown('Closing pause_physics service')
        self.unpause_physics.shutdown('Closing unpause_physics service')
        del self.name
        rospy.loginfo(self.name+" shutting down!")
    
    @property
    def get_name(self):
        return self.name

    @property
    def environment_shape(self):
        ''' Should return a tuple with the dimensions of the observation space '''
        return (3,)

    @property
    def action_shape(self):
        ''' Should return a tuple with the dimensions of the action space '''
        return (1,)
        