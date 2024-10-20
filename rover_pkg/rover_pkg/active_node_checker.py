import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# Known nodes are hard coded into this dictionary which is used to find node name in the json 
# file from the listed node name. The keys 0 to N-1 are related to their placements inside the JSON template!
known_node_names = {
    "ROVER": 0,
    "NAV_cmd_vel_manager": 1,
    "NAV_displacement_cmds": 2,
    "NAV_gamepad_interface": 3,
    "NavCSInterfacing": 4,
    "NAV_motor_cmds": 5,
    "SC_motor_cmds": 6,
    "DrillCSInterface": 7,
    "network_monitoring": 8,
}

class ActiveNodeChecker(Node):

    def __init__(self, json):
        super().__init__('HealthNode')
        self.json = json
        timer_period = 5 # seconds (we leave ROS some time to search, it may happen that the discovery is broken)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.trigger_launch = self.create_subscription(String, "trigger_system", self.trigger_launch_file, 10)

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
    
    def trigger_launch_file(self, msg):
        subsystem = msg.subsystem
        launch_file_name = msg.launch_file_name

        if subsystem == 'cameras':
            # trigger the cameras launch file