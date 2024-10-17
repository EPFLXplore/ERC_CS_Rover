import rclpy
from rclpy.node import Node

# Known nodes are hard coded into this dictionary which is used to find node name in the json file from the listed node name
known_node_names = {
    "ROVER": 0
}



class ActiveNodeChecker(Node):

    def __init__(self, json):
        super().__init__('HealthNode')
        self.json = json
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):

        # get_node_names returns a vector of all the available nodes in the ROS graph
        node_list = self.get_node_names()

        # For each name, pass it through a dictionary of known node name, 
        # if a match is found, modifiy the node status in the json 
        for name in known_node_names:
            # Check if name is in dictionary
            if name in node_list:
                self.json['rover']['software']['nodes'][known_node_names[name]]['status'] = True
            else:
                self.json['rover']['software']['nodes'][known_node_names[name]]['status'] = False