import rospy
from std_msgs.msg import Float32, String, Bool
from .auton_modules.state import SetIdle, State, StartPath, AutoBalance, Arm, Shooter

# The id of the auton, used for picking auton
auton_id = 8
auton_title = "1 Cone Engage"

# Start of our states
class Idle(SetIdle):
    """
    The state which waits for autonomous to start
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.setRobotPose()
        self.setIdle()

    def tick(self):
        return MoveFirstCone(self.ros_node)
  
class MoveFirstCone(Arm):
    def initialize(self):
        self.log_state()
        
    def execute_action(self):
        self.full_score_high()
        
    def tick(self):
        if self.get_arm_state() == "full_score_high":
            return RetractArm(self.ros_node)
        return self
    
class RetractArm(Arm):
    def initialize(self):
        self.log_state()
        
    def execute_action(self):
        self.retract()
        
    def tick(self):
        if self.get_arm_state() == "retract":
            return InsideBot(self.ros_node)
        return self
    
class InsideBot(Arm):
    def initialize(self):
        self.log_state()
        
    def execute_action(self):
        self.inside_bot()
        
    def tick(self):
        return Balance(self.ros_node)
    
class Balance(AutoBalance):
    def initialize(self):
        self.log_state()
        
    def execute_action(self):
        self.balance()
        
    def tick(self):
        if self.is_balanced():
            return Final(self.ros_node)
        return self

class Final(State):
    """
    The state which indicates that there are no limitations on device
    capabilities.
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        rospy.loginfo("END OF AUTON")

    def tick(self):
        return self

class Shutdown(SetIdle):
    """
    The state which indicates that there are no limitations on device
    capabilities.
    """

    def initialize(self):
        pass

    def execute_action(self):
        self.setIdle()

    def tick(self):
        return self

def start(ros_node):
    # Pick which topics to subscribe to
    ros_node.subscribe("/pathTable/status/path", Float32)
    ros_node.subscribe("/pathTable/status/finishedPath", String)
    ros_node.subscribe("/auto/balance/state", Bool)
    ros_node.subscribe("/auto/balance/r_balance", Bool)
    ros_node.subscribe("/auto/vision/set", String)

    # Return the wanted Start and Shutdown state
    return Idle, Shutdown
