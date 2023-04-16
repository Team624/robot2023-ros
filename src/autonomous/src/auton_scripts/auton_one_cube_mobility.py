import rospy
from std_msgs.msg import Float32, String, Bool
from .auton_modules.state import SetIdle, State, StartPath, AutoBalance, Arm, Shooter

# The id of the auton, used for picking auton
auton_id = 11
auton_title = "1 Cube Mobility"

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
        print("Running prime !")
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
  """
  The state which publishes the first path to follow
  """

  def initialize(self):
      self.log_state()

  def execute_action(self):
      self.start_paths(0)

  def tick(self):
      if self.finished_path(0):
        if self.should_balance():
          return Wait(self.ros_node)
        return Final(self.ros_node)
      return self

class Wait(State):
    def initialize(self):
      self.log_state()

    def execute_action(self):
      pass

    def tick(self):
        if self.check_timer(1):
            return Balance(self.ros_node)
        return self
    
class Balance(AutoBalance):
    def initialize(self):
        self.log_state()
        
    def execute_action(self):
        self.balance(reverse=True)
        
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
