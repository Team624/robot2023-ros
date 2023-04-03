import rospy
from std_msgs.msg import Float32, String, Bool
from .auton_modules.state import SetIdle, State, StartPath, Arm, AutoBalance, Shooter

# The id of the auton, used for picking auton
auton_id = 4
auton_title = "2 Piece Charger"
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
        return ScoreFirstCone(self.ros_node)

class ScoreFirstCone(Arm):
    def initialize(self):
        self.log_state()
        
    def execute_action(self):
        self.fast_score_high()
        
    def tick(self):
        if self.get_arm_state() == "fast_score_high":
            return StartFirstPath(self.ros_node)
        return self
    
class StartFirstPath(StartPath):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.start_paths(0, 1)

    def tick(self):
        return RetractArm(self.ros_node)
    
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
        if self.finished_path(0):
            return MoveIntakeCube(self.ros_node)
        return self
    
class MoveIntakeCube(Shooter):
    def initialize(self):
        self.log_state()
    def execute_action(self):
        self.intake()
    def tick(self):
        if self.finished_path(1):
            return PrimeCube(self.ros_node)
        return self
    
class PrimeCube(Shooter):
    def initialize(self):
        self.log_state()
    def execute_action(self):
        self.prime_low()
    def tick(self):
        return StartBalancePath(self.ros_node)
    
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
        self.balance(reverse=True)
        
    def tick(self):
        if self.is_balanced() and self.get_shooter_state() == "prime_low":
            return WaitForChargerStabalize(self.ros_node)
        return self
    
class WaitForChargerStabalize(State):
    def initialize(self):
        self.log_state()
        
    def execute_action(self):
        pass
        
    def tick(self):
        if self.check_timer(1.18):
            return ShootCube(self.ros_node)
        return self
    
class ShootCube(Shooter):
    def initialize(self):
        self.log_state()
    def execute_action(self):
        self.shoot_low()
    def tick(self):
        if (self.get_shooter_state() == "shoot_low"):
            return IdleShooter(self.ros_node)
        return self
    
class IdleShooter(Shooter):
    def initialize(self):
        self.log_state()
    def execute_action(self):
        self.idle()
    def tick(self):
        return Final(self.ros_node)

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
    ros_node.subscribe("/auto/shooter/state", String)

    # Return the wanted Start and Shutdown state
    return Idle, Shutdown
