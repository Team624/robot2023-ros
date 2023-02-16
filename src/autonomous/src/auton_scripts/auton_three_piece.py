import rospy
from std_msgs.msg import Float32, String, Bool
from .auton_modules.state import SetIdle, State, StartPath, AutoBalance

# The id of the auton, used for picking auton
auton_id = 2
auton_title = "3 Piece"

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
        return StartFirstPath(self.ros_node)

class StartFirstPath(StartPath):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.start_paths(0, 1, 2, 3, 4)

    def tick(self):
        if self.finished_path(4):
            if self.should_balance():
                return StartBalancePath(self.ros_node)
            return Final(self.ros_node)
        return self
    
class StartBalancePath(StartPath):
    def initialize(self):
        self.log_state()
    
    def execute_action(self):
        self.start_paths(5)
    
    def tick(self):
        if self.finished_path(5):
            return Balance(self.ros_node)
        return self
    
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
    ros_node.subscribe("/auto/balance/should_balance", Bool)

    # Return the wanted Start and Shutdown state
    return Idle, Shutdown
