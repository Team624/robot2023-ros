from geometry_msgs.msg import Point
from datetime import datetime
from autonomous.msg import Path


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
            path = AutoPath(len(self.paths), json_path)
            self.paths.append(path)
            
class AutoPath:
    def __init__(self, path_id, json_data):
        # Returns a tuple converts to list
        self.id = path_id
                
        self.time = json_data["time"]
        self.start_heading = json_data["start_heading"]
        self.end_heading = json_data["end_heading"]
        
        self.control_points = []
        
        for point in json_data["control_points"]:
            self.control_points.append((point["x"], point["y"]))
        
    def get_path(self):
        ''' Converts class into ros message '''
        
        path = Path()
        
        # Add data to the message
        path.path_index = self.id
        path.time = self.time
        
        path.start_heading = self.start_heading
        path.end_heading = self.end_heading
        
        point_msgs = []
        
        for point in self.control_points:
            point_msg = Point(point[0], point[1], 0)
            
            point_msgs.append(point_msg)
            
        path.control_points = point_msgs
        
        return path
