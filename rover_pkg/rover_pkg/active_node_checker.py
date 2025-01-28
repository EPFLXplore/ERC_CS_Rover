import rclpy
from rclpy.node import Node

known_node_names = {
    "ROVER": ("rover", "node_rover", True),
    "HealthNode": ("rover", "health_node", True),
    "NAV_cmd_vel_manager": ("navigation", "wheels_control", True),
    "NAV_displacement_cmds": ("navigation", "wheels_displacement", True),
    "NAV_gamepad_interface": ("navigation", "wheels_gamepad", True),
    "NAV_motor_cmds": ("navigation", "wheels_commands", True),
    "NavCSInterfacing": ("drill", "drill_interface", True),
    "drill_fsm_node": ("drill", "drill_fms", True),
    "SC_motor_cmds": ("drill", "drill_commands", True)
}

class ActiveNodeChecker(Node):

    def __init__(self, json, model):
        super().__init__('HealthNode')
        self.json = json
        self.model = model
        timer_period = 2 # seconds (we leave ROS some time to search, it may happen that the discovery is broken)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):

        # get_node_names returns a vector of all the available nodes in the ROS graph
        node_list = self.get_node_names()

        if "NAV_motor_cmds" not in node_list:
            self.json['rover']['status']['systems']['navigation']['status'] = 'Off'
            self.model.Nav.reset_informations()

        if "SC_motor_cmds" not in node_list:
            self.json['rover']['status']['systems']['drill']['status'] = 'Off'
            self.model.Drill.reset_informations()
        
        # TODO ADD HD

        # For each name, pass it through a dictionary of known node name, 
        # if a match is found, modifiy the node status in the json 
        for name in known_node_names:
            # Check if node in list corresponds to one in the dictionary
            if name in node_list:
                if known_node_names[name][2]:
                    self.json['rover']['software']['nodes'][known_node_names[name][0]][known_node_names[name][1]]['status'] = True
                else:
                    self.json['cameras'][known_node_names[name][0]][known_node_names[name][1]]['status'] = True
                
            # if the node is not found
            else:
                if known_node_names[name][2]:
                    self.json['rover']['software']['nodes'][known_node_names[name][0]][known_node_names[name][1]]['status'] = False
                else:
                    self.json['cameras'][known_node_names[name][0]][known_node_names[name][1]]['status'] = False
