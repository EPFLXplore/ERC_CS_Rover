import rclpy
from rclpy.node import Node
import time

known_node_names = {
    "ROVER": ("rover", "node_rover", True),
    "HealthNode": ("rover", "health_node", True),
    "NAV_cmd_vel_manager": ("navigation", "wheels_control", True),
    "NAV_displacement_cmds": ("navigation", "wheels_displacement", True),
    "NAV_gamepad_interface": ("navigation", "wheels_gamepad", True),
    "NAV_motor_cmds": ("navigation", "wheels_commands", True),
    "NavCSInterfacing": ("navigation", "navigation_interface", True),

    "DrillCSInterface": ("drill", "drill_fms", True),
    "SC_motor_cmds": ("drill", "drill_commands", True),

    "HDCSInterfacing": ("handling_device", "handling_device_interface", True),
    "MotorController": ("handling_device", "motor_control", True),
    "kinematics_task_executor": ("handling_device", "task_executor", True),

    "/ROVER/camera_cs_0": ("control_station", "Behind", False),
    "/ROVER/camera_cs_1": ("control_station", "Left", False),
    "/ROVER/camera_cs_2": ("control_station", "Right", False),
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
            #self.rover_node.model.Elec.send_led_commands("navigation", "Off")
            self.json['rover']['status']['systems']['navigation']['status'] = 'Off'
            self.model.Nav.reset_informations()

        if "SC_motor_cmds" not in node_list:
            self.json['rover']['status']['systems']['drill']['status'] = 'Off'
            self.model.Drill.reset_informations()
            
        if "HDCSInterfacing" not in node_list:
            self.json['rover']['status']['systems']['handling_device']['status'] = 'Off'
            self.model.HD.reset_informations()
        
        # TODO ADD HD

        # For each name, pass it through a dictionary of known node name, 
        # if a match is found, modifiy the node status in the json 
        for name in known_node_names:

            # Check if node in list corresponds to one in the dictionary
            if name in node_list:
                
                # The nodes of the cameras are not in software entry of JSON. We differentiate that with this if
                if known_node_names[name][2]:
                    self.json['rover']['software']['nodes'][known_node_names[name][0]][known_node_names[name][1]]['status'] = True
                else:
                    self.json['cameras'][known_node_names[name][0]][known_node_names[name][1]]['node'] = True
                
            # if the node is not found
            else:
                if known_node_names[name][2]:
                    self.json['rover']['software']['nodes'][known_node_names[name][0]][known_node_names[name][1]]['status'] = False
                else:
                    self.json['cameras'][known_node_names[name][0]][known_node_names[name][1]]['node'] = False
