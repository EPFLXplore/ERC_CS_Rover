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
    "NavCSInterfacing": ("drill", "drill_interface", True),
    "drill_fsm_node": ("drill", "drill_fms", True),
    "SC_motor_cmds": ("drill", "drill_commands", True),

    "/ROVER/camera_cs_0": ("control_station", "Behind", False)
}

class ActiveNodeChecker(Node):

    def __init__(self, json):
        super().__init__('HealthNode')
        self.json = json
        timer_period = 3 # seconds (we leave ROS some time to search, it may happen that the discovery is broken)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):

        # get_node_names returns a vector of all the available nodes in the ROS graph
        node_list = self.get_node_names()

        if "NAV_motor_cmds" not in node_list:
            i = 0
            while i != 2:
                time.sleep(1)
                if "NAV_motor_cmds" not in node_list:
                    i += 1
                else:
                    break
            #self.rover_node.model.Elec.send_led_commands("navigation", "Off")

            self.json['rover']['status']['systems']['navigation']['status'] = 'Off'


            # front_left wheel
            self.json['navigation']['wheels']['front_left']['current_driving'] = "0.0"
            self.json['navigation']['wheels']['front_left']['current_steering'] = "0.0"
            self.json['navigation']['wheels']['front_left']['speed'] = "0.0"
            self.json['navigation']['wheels']['front_left']['steering_angle'] = "0.0"
            self.json['navigation']['wheels']['front_left']['steering_motor_state'] = False
            self.json['navigation']['wheels']['front_left']['driving_motor_state'] = False
            self.json['navigation']['wheels']['front_left']['steering_fault'] = False
            self.json['navigation']['wheels']['front_left']['driving_fault'] = False
            

            # front_right wheel
            self.json['navigation']['wheels']['front_right']['current_driving'] = "0.0"
            self.json['navigation']['wheels']['front_right']['current_steering'] = "0.0"
            self.json['navigation']['wheels']['front_right']['speed'] = "0.0"
            self.json['navigation']['wheels']['front_right']['steering_angle'] = "0.0"
            self.json['navigation']['wheels']['front_right']['steering_motor_state'] = False
            self.json['navigation']['wheels']['front_right']['driving_motor_state'] = False
            self.json['navigation']['wheels']['front_right']['steering_fault'] = False
            self.json['navigation']['wheels']['front_right']['driving_fault'] = False

            
            # back_right wheel
            self.json['navigation']['wheels']['rear_right']['current_driving'] = "0.0"
            self.json['navigation']['wheels']['rear_right']['current_steering'] = "0.0"
            self.json['navigation']['wheels']['rear_right']['speed'] = "0.0"
            self.json['navigation']['wheels']['rear_right']['steering_angle'] = "0.0"
            self.json['navigation']['wheels']['rear_right']['steering_motor_state'] = False
            self.json['navigation']['wheels']['rear_right']['driving_motor_state'] = False
            self.json['navigation']['wheels']['rear_right']['steering_fault'] = False
            self.json['navigation']['wheels']['rear_right']['driving_fault'] = False

            
            # back_left wheel
            self.json['navigation']['wheels']['rear_left']['current_driving'] = "0.0"
            self.json['navigation']['wheels']['rear_left']['current_steering'] = "0.0"
            self.json['navigation']['wheels']['rear_left']['speed'] = "0.0"
            self.json['navigation']['wheels']['rear_left']['steering_angle'] = "0.0"
            self.json['navigation']['wheels']['rear_left']['steering_motor_state'] = False
            self.json['navigation']['wheels']['rear_left']['driving_motor_state'] = False
            self.json['navigation']['wheels']['rear_left']['steering_fault'] = False
            self.json['navigation']['wheels']['rear_left']['driving_fault'] = False

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
