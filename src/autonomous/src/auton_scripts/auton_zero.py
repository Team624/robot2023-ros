import rospy
from std_msgs.msg import Float32, String
from .auton_modules.state import SetIdle, State, StartPath

# The id of the auton, used for picking auton
auton_id = 1
auton_title = "Auton Zero"

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
        self.start_path(0)

    def execute_action(self):
        pass

    def tick(self):
        if self.finished_path(0):
            return StartSecondPath(self.ros_node)
        return self

class StartSecondPath(StartPath):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()
        self.start_path(1)

    def execute_action(self):
        pass
        
    def tick(self):
        if self.finished_path(1):
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

    # Return the wanted Start and Shutdown state
    return Idle, Shutdown
