import json
import rospy
from std_msgs.msg import Float32, Float32MultiArray, Bool, String
from .path import Autons
import time
import rospkg 
import math
from autonomous.msg import PathStart
import random

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

        return (string_array[0] == "true" and int(string_array[1]) == path_num) or int(string_array[1]) > path_num
        
    def is_balanced(self):
        return self.ros_node.get_data("/auto/balance/state")
    
    def should_balance(self):
        return self.ros_node.get_data("/auto/balance/should_balance")
    
    def get_arm_state(self):
        return self.ros_node.get_data("/auto/arm/state")
    
    def get_shooter_state(self):
        return self.ros_node.get_data("/auto/shooter/state")
    
    def is_vision_aligned(self):
        return self.ros_node.get_data("/auto/vision/state")

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
        msg = PathStart()
        msg.path_indexes = []
        
        self.ros_node.publish("/pathTable/startPathIndex", PathStart, msg, latching = True)
        self.ros_node.publish("/auto/balance/set", String, "false false", latching = True)
        self.ros_node.publish("/auto/arm/set", String, "none", latching = True)
        self.ros_node.publish("/auto/vision/set", String, "none", latching = True)
        self.ros_node.publish("/auto/shooter/set", String, "none")

class StartPath(State):

    # Actions
    def start_paths(self, *indexes):
        msg = PathStart()
        msg.path_indexes = list(indexes)
        
        self.ros_node.publish("/pathTable/startPathIndex", PathStart, msg, latching = True)

class AutoBalance(State):
    def balance(self, reverse = False):
        self.ros_node.publish("/auto/balance/set", String, "true " + str(reverse).lower(), latching = True)
        
class Arm(State):
    def tipped_intake(self):
        self.ros_node.publish("/auto/arm/set", String, "tipped_intake", latching = True)
    def standing_intake(self):
        self.ros_node.publish("/auto/arm/set", String, "standing_intake", latching = True)
        
    def full_score_high(self):
        self.ros_node.publish("/auto/arm/set", String, "full_score_high", latching = True)
    def full_score_mid(self):
        self.ros_node.publish("/auto/arm/set", String, "full_score_mid", latching = True)
    def full_score_low(self):
        self.ros_node.publish("/auto/arm/set", String, "full_score_low", latching = True)
        
    def fast_score_high(self):
        self.ros_node.publish("/auto/arm/set", String, "fast_score_high", latching = True)
    def fast_score_mid(self):
        self.ros_node.publish("/auto/arm/set", String, "fast_score_mid", latching = True)
    
    def pre_score_high(self):
        self.ros_node.publish("/auto/arm/set", String, "pre_score_high", latching = True)
    def pre_score_mid(self):
        self.ros_node.publish("/auto/arm/set", String, "pre_score_mid", latching = True)
        
    def finish_score_high(self):
        self.ros_node.publish("/auto/arm/set", String, "finish_score_high", latching = True)
    def finish_score_mid(self):
        self.ros_node.publish("/auto/arm/set", String, "finish_score_mid", latching = True)
        
    def retract(self):
        self.ros_node.publish("/auto/arm/set", String, "retract", latching = True)
    def inside_bot(self):
        self.ros_node.publish("/auto/arm/set", String, "inside_bot", latching = True)
        
    def slow_mode(self, slow):
        self.ros_node.publish("/auto/arm/slow", Bool, slow, latching = True)

class Vision(State):
    def align_cone(self):
        self.ros_node.publish("/auto/vision/set", String, "cone" + str(random.randint(0, 1000000)), latching = True)
        
    def align_cube(self, grid):
        self.ros_node.publish("/auto/vision/set", String, "cube " + grid, latching = True)
        
class Shooter(State):
    def prime_high_over_bump(self):
        self.ros_node.publish("/auto/shooter/set", String, "prime_high_over_bump", latching=True)
    def prime_high_flat_community(self):
        self.ros_node.publish("/auto/shooter/set", String, "prime_high_flat_community", latching=True)
    def prime_high_charger(self):
        self.ros_node.publish("/auto/shooter/set", String, "prime_high_charger", latching=True)
    def prime_high(self):
        self.ros_node.publish("/auto/shooter/set", String, "prime_high", latching=True)
    def prime_mid(self):
        self.ros_node.publish("/auto/shooter/set", String, "prime_mid", latching=True)
    def prime_low(self):
        self.ros_node.publish("/auto/shooter/set", String, "prime_low", latching=True)
    def intake(self):
        self.ros_node.publish("/auto/shooter/set", String, "deploy_intake", latching=True)
    def shoot_high_over_bump(self):
        self.ros_node.publish("/auto/shooter/set", String, "shoot_high_over_bump", latching=True)
    def shoot_high_flat_community(self):
        self.ros_node.publish("/auto/shooter/set", String, "shoot_high_flat_community", latching=True)
    def shoot_high_charger(self):
        self.ros_node.publish("/auto/shooter/set", String, "shoot_high_charger", latching=True)
    def shoot_high(self):
        self.ros_node.publish("/auto/shooter/set", String, "shoot_high", latching=True)
    def shoot_mid(self):
        self.ros_node.publish("/auto/shooter/set", String, "shoot_mid", latching=True)
    def shoot_low(self):
        self.ros_node.publish("/auto/shooter/set", String, "shoot_low", latching=True)
    def idle(self):
        self.ros_node.publish("/auto/shooter/set", String, "idle", latching=True)
    