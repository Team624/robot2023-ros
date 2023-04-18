import rospy
from std_msgs.msg import Float32, String, Bool
from .auton_modules.state import SetIdle, State, StartPath, AutoBalance, Arm, Shooter, Vision

# The id of the auton, used for picking auton
auton_id = 13
auton_title = "3 Piece Flat Modified"

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
        self.start_paths(0)

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
        if self.get_arm_state() == "inside":
            return MoveIntakeFirstCube(self.ros_node)
        return self

class MoveIntakeFirstCube(Shooter):
    def initialize(self):
        self.log_state()
    def execute_action(self):
        self.intake()
    def tick(self):
        if self.finished_path(0) and self.get_shooter_state() == "intake":
            return StartSecondPath(self.ros_node)
        return self

class StartSecondPath(StartPath):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.start_paths(1)

    def tick(self):
        if self.check_timer(2):
            return PrimeShooterHigh(self.ros_node)
        return self
class PrimeShooterHigh(Shooter):
    def initialize(self):
        self.log_state()
        
    def execute_action(self):
        self.prime_high()
        
    def tick(self):
        if self.get_shooter_state() == "prime_high" and self.finished_path(1):
            return ShootHigh(self.ros_node)
        return self
class ShootHigh(Shooter):
    def initialize(self):
        self.log_state()
        
    def execute_action(self):
        self.shoot_high()
        
    def tick(self):
        if self.get_shooter_state() == "shoot_high":
            return MoveIntakeSecondCube(self.ros_node)
        return self  

class MoveIntakeSecondCube(Shooter):
    def initialize(self):
        self.log_state()
    def execute_action(self):
        self.intake()
    def tick(self):
        return StartThirdPath(self.ros_node)
 

class StartThirdPath(StartPath):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.start_paths(2)

    def tick(self):
        if self.finished_path(2) and self.get_shooter_state()=="intake":
            return StartFourthPath(self.ros_node)
        return self
class StartFourthPath(StartPath):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.start_paths(3)

    def tick(self):
        if self.check_timer(2):
            return PrimeShooterMid(self.ros_node)
        return self
class PrimeShooterMid(Shooter):
    def initialize(self):
        self.log_state()
        
    def execute_action(self):
        self.prime_mid()
        
    def tick(self):
        if self.get_shooter_state() == "prime_high" and self.finished_path(3):
            return ShootHigh(self.ros_node)
        return self

class ShootMid(Shooter):
    def initialize(self):
        self.log_state()
        
    def execute_action(self):
        self.shoot_mid()
        
    def tick(self):
        if self.get_shooter_state() == "shoot_mid":
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

    # Return the wanted Start and Shutdown state
    return Idle, Shutdown
