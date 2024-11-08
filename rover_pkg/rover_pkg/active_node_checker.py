import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import subprocess

# Known nodes are hard coded into this dictionary which is used to find node name in the json file from the listed node name
# The first argument of the dictionary output is a bool to say if the node is a camera or not
# if a node is in "software" in the json file, only change the active status, second argument of the tuple is the index of the node in the json "software"
# if a node is a camera, change the bandwidth value
known_node_names = {
    "ROVER": (False, 0), # is not camera
    "NAV_cmd_vel_manager": (False, 1),
    "NAV_displacement_cmds": (False, 2),
    "NAV_gamepad_interface": (False, 3),
    "NavCSInterfacing": (False, 4),
    "NAV_motor_cmds": (False, 5),
    "SC_motor_cmds": (False, 6),
    "DrillCSInterface": (False, 7),
    "network_monitoring": (False, 8),
    "camera_cs_0": (True, 'control_station', 'Front'), # ITS A CAMERAC + NOT SHOWING NAMESPACE!
}

class ActiveNodeChecker(Node):

    def __init__(self, json):
        super().__init__('HealthNode')
        self.json = json
        timer_period = 5 # seconds (we leave ROS some time to search, it may happen that the discovery is broken)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):

        # get_node_names returns a vector of all the available nodes in the ROS graph
        node_list = self.get_node_names()

        print(node_list)

        # For each name, pass it through a dictionary of known node name, 
        # if a match is found, modifiy the node status in the json 
        for name in known_node_names:
            # Check if node in list corresponds to one in the dictionary
            if name in node_list:
                # if the node is found and is a camera find the node bandwidth and modify the value in the json file
                if known_node_names[name][0]:
                    self.get_logger().info("adéfjbnjerbgjiénk")
                    # Use subprocess to find the BW of the camera
                    command = f"ros2 topic bw {name} --once"
                    # subprocess.run runs the given command, it retains the output if capture_output = True, it converts the output to text ratehr then
                    # Binary if text=True, and runs the command in shell if shell = True
                    result = subprocess.run(command, capture_output=True, text=True, shell=True)
                    # topic bw returns a mess of a message, we need to extract only the bw rate
                    print(result.split(" "))
                    print(result)
                    self.json['rover']['cameras'][known_node_names[name][1]][known_node_names[name][2]]['data_rate'] = result
                
                # if node is active but is not a camera, chage its active status to true
                else:
                    self.json['rover']['software']['nodes'][known_node_names[name][1]]['status'] = True
                
            # if the node is not found
            else:
                # if the node is not active and is a camera, set it's bandwidth to 0 in the json file
                if known_node_names[name][0]:
                    self.json['cameras'][known_node_names[name][1]][known_node_names[name][0][2]]['data_rate'] = 0

                # if the node is not active and is not a camera, set it's status to false in the json file
                else:
                    self.json['rover']['software']['nodes'][known_node_names[name][1]]['status'] = False
