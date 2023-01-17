from std_msgs.msg import Float32, Float64, Bool
from geometry_msgs.msg import Pose
from datetime import datetime
import json
from autonomous.msg import GoalPath, Goal


class Autons:
    def __init__(self, auto_id, title="N/A", start_pose=[0,0,0], description="N/A", paths=[], date_created=str(datetime.utcnow())):
        # Header Info
        self.id = auto_id
        self.title = title
        self.start_pose = start_pose
        self.description = description
        self.date_created = date_created[0:date_created.index(' ')]
        
        # Main data
        self.paths = paths

    def deserialize_json(self, json_data):
        """ This formats data to be saved to this server """
        self.id = json_data["id"] 
        self.title = json_data["title"]
        self.start_pose = json_data["start_pose"]
        self.description = json_data["description"]

        # Loop through all the paths and update the class
        self.paths = []
        for json_path in json_data["paths"]:
            goals = []
            for json_goal in json_path["goals"]:
                goal = AutoGoal(json_goal["x"], json_goal["y"], json_goal["t"])
                goals.append(goal)
            path = AutoPath(len(self.paths), 
                         goals,
                         json_path["time"],
                         json_path["start_heading"], 
                         json_path["end_heading"], 
                         json_path["tolerance"],
                         json_path["name"], 
                         end_of_path_stop=json_path["end_of_path_stop"])
            
            self.paths.append(path)
            
class AutoPath:
    def __init__(self, path_id, goals, time, start_heading, end_heading, tolerance, name, end_of_path_stop = True):
        # Returns a tuple converts to list
        self.id = path_id
        self.name = name
        self.time = time

        self.max_linear_acceleration = 1E9
        self.end_of_path_stop = end_of_path_stop
        
        self.start_heading = start_heading
        self.end_heading = end_heading

        self.tolerance = tolerance
        self.goals = goals

    def get_path(self):
        ''' Converts class into ros message '''
        
        goal_path = GoalPath()
        
        # Convert every goal into a message
        msg_goals = []
        for goal in self.goals:
            msg_goals.append(goal.get_goal())
        
        # Add data to the message
        goal_path.goals = msg_goals
        goal_path.time = self.time
        goal_path.start_heading = self.start_heading
        goal_path.end_heading = self.end_heading
        goal_path.end_of_path_stop = self.end_of_path_stop
        goal_path.tolerance = self.tolerance

        return goal_path

class AutoGoal:
    def __init__(self, x, y, t):
        self.x = x
        self.y = y

        self.t = t

    def get_goal(self):
        goal = Goal()

        pose = Pose()
        pose.position.x = self.x
        pose.position.y = self.y

        goal.pose = pose
        return goal