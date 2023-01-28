import json
import rospy
from std_msgs.msg import Float32, Float32MultiArray
from .path import Autons
import time
import rospkg 
import math

global data
data = []

def read_json():
    """ This reads the auton data and saves it to a list to be used """
    try:
        rospack = rospkg.RosPack()
        file_path = rospack.get_path('autonomous') + "/src/auton_scripts/auton_modules/path-editor/data.json"
        with open(file_path) as json_file:
            json_data = json.load(json_file)
            
            new_data = []
            for d in json_data:
                a = Autons(len(new_data))
                a.deserialize_json(d)
                new_data.append(a)

            global data
            data = new_data
    except Exception as e:
        print("Failed to decode data file: ", str(e))
        
read_json()

class State(object):
    """
    We define a state object which provides some utility functions for the
    individual states within the state machine.
    """

    def __init__(self, ros_node):
        self.ros_node = ros_node
        self.initialize()
        self.action_executed = False

        self.start_time = time.time()

    def log_state(self):
        """ Logs the name of the State """
        rospy.loginfo("STATE: %s   [%s]" %(self.__class__.__name__, 15 - self.ros_node.get_time()))
    
    # Counts the amount of time 
    def start_timer(self):
        """ Used to start a timer that counts time within a state """
        self.start_time = time.time()

    def check_timer(self, wanted_time):
        """ Checks if the amount of time given has passed """
        if time.time() - self.start_time >= wanted_time:
            return True
        return False

    # Function for working with wrapping angles
    def wrap_angle(self, angle):
        if angle < 0.0:
            return (math.pi * 2) + angle
        elif angle >= (math.pi * 2):
            return angle - (math.pi * 2)

        return angle
    
    # Get data
    def get_path(self):
        return self.ros_node.get_data("/pathTable/status/path")

    def get_point(self):
        return self.ros_node.get_data("/pathTable/status/point")

    def finished_path(self, path_num):
        string_array = self.ros_node.get_data("/pathTable/status/finishedPath").split()

        if (string_array[0] == "true" and int(string_array[1]) >= path_num):
            return True
        else:
            return False

    # This runs in the child class when created
    def initialize(self):
        pass

    # This runs once in the child class
    def execute_action(self):
        pass

    # This runs in a loop in the child class
    def tick(self):
        pass

    # This makes it so that the functions created in the child class act as they should
    def update(self):
        if not self.action_executed:
            self.execute_action()
            self.action_executed = True
        return self.tick()

class SetIdle(State):

    def setRobotPose(self):
        global data
        msg = Float32MultiArray()
        for auton in data:
            if auton.title == self.ros_node.auton_title:
                msg.data = auton.start_pose
                self.ros_node.publish('/robot_set_pose', Float32MultiArray, msg, latching = True)
                rospy.loginfo("Reset Robot Pose")

    def setIdle(self):
        # Path Idle
        self.ros_node.publish("/pathTable/startPathIndex", Float32, -1, latching = True)



class StartPath(State):

    # Actions
    def start_path(self, index):
        """ This gets the path data from the json file and publishes to diff_drive """
        # Checks for updated data
        self.ros_node.publish("/pathTable/startPathIndex", Float32, index, latching = True)
