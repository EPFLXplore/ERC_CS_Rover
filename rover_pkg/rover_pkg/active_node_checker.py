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
    "perception_node": ("handling_device", "perception", True),
    
    "costco_publisher": ("electronics", "avionics", True),

    "/ROVER/camera_cs_0": ("rover", "Behind", False),
    "/ROVER/camera_cs_1": ("rover", "Left", False),
    "/ROVER/camera_cs_2": ("rover", "Drill", False),
    "/ROVER/camera_cs_3": ("rover", "Right", False),
    
    "/NAV/camera_nav_front": ("navigation", "Front", False),
    "/NAV/camera_aruco_left": ("navigation", "Up1", False),
    "/NAV/camera_aruco_right": ("navigation", "Up2", False),
    
    "/HD/camera_hd_gripper": ("handling_device", "Gripper", False),
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
            
        if "HDCSInterfacing" not in node_list:
            self.json['rover']['status']['systems']['handling_device']['status'] = 'Off'
            self.model.HD.reset_informations()
            
        # if "costco_publisher" not in node_list:
        #     self.model.Elec.reset_informations()
        
        '''
        For each name, pass it through a dictionary of known node name, 
        if a match is found, modifiy the node status in the json
        
        Be aware that, for Cameras, we have the status and the node in the rover state. The 'status', for a camera activated,
        checks that the camera is publishing. The 'node' is another boolean saying if the node is actually running, so without
        errors. 
        '''
        for name in known_node_names:

            # Check if node in list corresponds to one in the dictionary
            if name in node_list:
                
                # The nodes of the cameras are not in software entry of JSON. We differentiate that with this if
                if known_node_names[name][2]:
                    self.json['rover']['software']['nodes'][known_node_names[name][0]][known_node_names[name][1]]['status'] = True
                else:
                    self.json['cameras'][known_node_names[name][0]][known_node_names[name][1]]['node'] = True
                    self.json['cameras'][known_node_names[name][0]][known_node_names[name][1]]['status'] = True
                
            # if the node is not found
            else:
                if known_node_names[name][2]:
                    self.json['rover']['software']['nodes'][known_node_names[name][0]][known_node_names[name][1]]['status'] = False
                else:
                    self.json['cameras'][known_node_names[name][0]][known_node_names[name][1]]['node'] = False
                    self.json['cameras'][known_node_names[name][0]][known_node_names[name][1]]['status'] = False
