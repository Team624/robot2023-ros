import rospy
from std_msgs.msg import Float32, String, Bool
from .auton_modules.state import SetIdle, State, StartPath, Arm, AutoBalance, Intake

# The id of the auton, used for picking auton
auton_id = 7
auton_title = "Real Two Piece Blue"

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
        return MoveArmCone(self.ros_node)
    
class MoveArmCone(Arm):
    def initialize(self):
        self.log_state()
    
    def execute_action(self):
        self.move_cone_high()
    
    def tick(self):
        if self.get_arm_state() == "high":
            return TelescopeZero(self.ros_node)
        return self


class TelescopeZero(Arm):
    def initialize(self):
        self.log_state()
    
    def execute_action(self):
        self.retract()
    
    def tick(self):
        if self.get_arm_state() == "retract": 
            return MoveArmIntake(self.ros_node)
        return self
        

class MoveArmIntake(Arm):
    def initialize(self):
        self.log_state()
    def execute_action(self):
        self.move_intake()
    def tick(self):
        return RunIntake(self.ros_node)
    
class RunIntake(Intake):
    def initialize(self):
        self.log_state()
    def execute_action(self):
        self.run_intake()
    def tick(self):
        return StartFirstPath(self.ros_node)
    
class StartFirstPath(StartPath):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.start_paths(0)

    def tick(self):
        if self.finished_path(0):
            return IdleIntake(self.ros_node)
        return self

class IdleIntake(Intake):
    def initialize(self):
        self.log_state()
    def execute_action(self):
        self.idle_intake()
    def tick(self):
        return TelescopeZero1(self.ros_node)

class TelescopeZero1(Arm):
    def initialize(self):
        self.log_state()
    
    def execute_action(self):
        self.retract()
    
    def tick(self):
        if self.get_arm_state() == "retract" and self.check_timer(1):
            return MoveArmCube(self.ros_node)
        return self

class MoveArmCube(Arm):
    def initialize(self):
        self.log_state()
    def execute_action(self):
        self.move_cube_high()
    def tick(self):
        return StartSecondPath(self.ros_node)
    
class StartSecondPath(StartPath):
    def initialize(self):
        self.log_state()
    def execute_action(self):
        self.start_paths(1)
    def tick(self):
        if self.finished_path(1) and self.get_arm_state() == "high":
            return ReverseCube(self.ros_node)
        return self
        
class ReverseCube(Intake):
    def initialize(self):
        self.log_state()
    def execute_action(self):
        self.reverse_cube()
    def tick(self):
        if self.check_timer(0.5):
            return FunnelArm(self.ros_node)
        return self
            
class FunnelArm(Arm):
    def initialize(self):
        self.log_state()
        
    def execute_action(self):
        self.retract()
        
    def tick(self):
        if self.get_arm_state() == "retract":
            if self.should_balance():
                return StartThirdPath(self.ros_node)
            return Final(self.ros_node)
        return self    

class StartThirdPath(StartPath):
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
