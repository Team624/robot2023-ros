import rospy
from std_msgs.msg import Float32, String, Bool
from .auton_modules.state import SetIdle, State, StartPath, Arm, AutoBalance

# The id of the auton, used for picking auton
auton_id = 6
auton_title = "2 Piece Right"

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
            return MoveFirstCone(self.ros_node)
        return self

class MoveFirstCone(Arm):
    def initialize(self):
        self.log_state()
        
    def execute_action(self):
        self.move_cone_high()
        
    def tick(self):
        print(self.get_arm_state())
        if self.get_arm_state() == "high":
            return PlaceFirstCone(self.ros_node)
        return self
        
class PlaceFirstCone(Arm):
    def initialize(self):
        self.log_state()
        
    def execute_action(self):
        self.place()
        
    def tick(self):
        if self.check_timer(0.3):
            return MoveIntakeCube(self.ros_node)
        return self

class MoveIntakeCube(StartPath, Arm):
    def initialize(self):
        self.log_state()
    def execute_action(self):
        self.start_paths(0)
        self.move_intake()
    def tick(self):
        if self.finished_path(0) and self.get_arm_state() == "intake":
            return IntakeCube(self.ros_node)
        return self
        
class IntakeCube(Arm):
    def initialize(self):
        self.log_state()
    def execute_action(self):
        self.intake()
    def tick(self):
        if (self.check_timer(0.5)):
            return MovePlaceCube(self.ros_node)
        return self
    
class MovePlaceCube(StartPath, Arm):
    def initialize(self):
        self.log_state()
    def execute_action(self):
        self.move_cube_high()
        self.start_paths(1)
    def tick(self):
        if (self.finished_path(1) and self.get_arm_state() == "cube_high"):
            return PlaceCube(self.ros_node)
        return PlaceCube(self.ros_node)
    
class PlaceCube(Arm):
    def initialize(self):
        self.log_state()
    def execute_action(self):
        self.place()
    def tick(self):
        if (self.check_timer(0.3)):
            if (self.should_balance()):
                return StartBalancePath(self.ros_node)
            return Final(self.ros_node)
        return self
      
class StartBalancePath(StartPath):
    def initialize(self):
        self.log_state()
    
    def execute_action(self):
        self.start_paths(2)
    
    def tick(self):
        if self.finished_path(2):
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
    ros_node.subscribe("/auto/arm/state", String)
    ros_node.subscribe("/auto/vision/state", String)

    # Return the wanted Start and Shutdown state
    return Idle, Shutdown
