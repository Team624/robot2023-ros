import rospy
from std_msgs.msg import Float32, String, Bool
from .auton_modules.state import SetIdle, State, StartPath, AutoBalance, Arm, Shooter, Vision

# The id of the auton, used for picking auton
auton_id = 5
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
        return PrimeShooter(self.ros_node)
    
class PrimeShooter(Shooter):
    def initialize(self):
        self.log_state()
        
    def execute_action(self):
        self.prime_high()
        
    def tick(self):
        if self.get_shooter_state() == "prime_high":
            return ShootHigh(self.ros_node)
        return self
        
class ShootHigh(Shooter):
    def initialize(self):
        self.log_state()
        
    def execute_action(self):
        self.shoot_high()
        
    def tick(self):
        if self.get_shooter_state() == "shoot_high":
            return IdleShooter(self.ros_node)
        return self
    
class IdleShooter(Shooter):
    def initialize(self):
        self.log_state()
    def execute_action(self):
        self.idle()
    def tick(self):
        return StartFirstPath(self.ros_node)
    
class StartFirstPath(StartPath):
    def initialize(self):
        self.log_state()
    def execute_action(self):
        self.start_paths(0)
    def tick(self):
        if self.get_shooter_state() == "idle":
            return IntakeCone(self.ros_node)
        return self
class IntakeCone(Arm):
    def initialize(self):
        self.log_state()
    def execute_action(self):
        self.move_intake()
    def tick(self):
        if self.finished_path(0):
            return InsideBotArm(self.ros_node)
        return self
    
class InsideBotArm(Arm):
    def initialize(self):
        self.log_state()
    def execute_action(self):
        self.inside_bot()
    def tick(self):
        return StartSecondPath(self.ros_node)
    
class StartSecondPath(StartPath):
    def initialize(self):
        self.log_state()
    def execute_action(self):
        self.start_paths(1)
    def tick(self):
        if self.finished_path(1):
            return AlignFirstCone(self.ros_node)
        return self
        
class AlignFirstCone(Vision):
    def initialize(self):
        self.log_state()
    def execute_action(self):
        self.align_cone()
    def tick(self):
        if self.is_vision_aligned():
            return MoveConeHigh(self.ros_node)
        return self
    
class MoveConeHigh(Arm):
    def initialize(self):
        self.log_state()
    def execute_action(self):
        self.move_cone_high()
    def tick(self):
        if self.get_arm_state() == "high":
            return RetractFirstCone(self.ros_node)
        return self
        
class RetractFirstCone(Arm):
    def initialize(self):
        self.log_state()
    def execute_action(self):
        self.retract()
    def tick(self):
        if self.get_arm_state() == "retract":
            return StartThirdPath(self.ros_node)
        return self
    
class StartThirdPath(StartPath):
    def initialize(self):
        self.log_state()
    def execute_action(self):
        self.start_paths(2, 3)
    def tick(self):
        return MoveSecondIntake(self.ros_node)
    
class MoveSecondIntake(Arm):
    def initialize(self):
        self.log_state()
    def execute_action(self):
        self.move_intake()
    def tick(self):
        if self.get_arm_state() == "intake" and self.finished_path(3):
            return StartFourthPath(self.ros_node)
        return self
    
class StartFourthPath(StartPath):
    def initialize(self):
        self.log_state()
    def execute_action(self):
        self.start_paths(4)
    def tick(self):
        return SecondInsideBot(self.ros_node)
    
class SecondInsideBot(Arm):
    def initialize(self):
        self.log_state()
    def execute_action(self):
        self.inside_bot()
    def tick(self):
        if self.finished_path(4):
            return AlignSecondCone(self.ros_node)
        return self
    
class AlignSecondCone(Vision):
    def initialize(self):
        self.log_state()
    def execute_action(self):
        self.align_cone()
    def tick(self):
        if self.is_vision_aligned():
            return MoveSecondConeHigh(self.ros_node)
        return self
    
class MoveSecondConeHigh(Arm):
    def initialize(self):
        self.log_state()
    def execute_action(self):
        self.move_cone_high()
    def tick(self):
        if self.get_arm_state() == "high":
            return RetractSecondCone(self.ros_node)
        return self
        
class RetractSecondCone(Arm):
    def initialize(self):
        self.log_state()
    def execute_action(self):
        self.retract()
    def tick(self):
        if self.get_arm_state() == "retract":
            return FinalInsideBot(self.ros_node)
        return self

class FinalInsideBot(Arm):
    def initialize(self):
        self.log_state()
    def execute_action(self):
        self.inside_bot()
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

    # Return the wanted Start and Shutdown state
    return Idle, Shutdown
