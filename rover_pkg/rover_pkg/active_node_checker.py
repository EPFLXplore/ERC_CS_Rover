from rclpy.node import Node

'''
Author: Giovanni Ranieri & Matas Jones
Year: 2024-25
Description: This node determines if nodes are running, and updates the JSON accordingly.
'''

known_node_names = {
    
    # Rover nodes
    "ROVER": ("rover", "node_rover", True),
    "HealthNode": ("rover", "health_node", True),
    
    # Navigation nodes
    "NAV_cmd_vel_manager": ("navigation", "wheels_control", True),
    "NAV_displacement_cmds": ("navigation", "wheels_displacement", True),
    "NAV_gamepad_interface": ("navigation", "wheels_gamepad", True),
    "NAV_motor_cmds": ("navigation", "wheels_commands", True),
    "NavCSInterfacing": ("navigation", "navigation_interface", True),

    # Drill nodes
    "SC_motor_cmds": ("drill", "drill_commands", True),

    # Handling Device nodes
    "HDCSInterfacing": ("handling_device", "handling_device_interface", True),
    "MotorController": ("handling_device", "motor_control", True),
    "kinematics_task_executor": ("handling_device", "task_executor", True),
    "perception_node": ("handling_device", "perception", True),
    
    # Electronics nodes
    "avionics_nexus": ("electronics", "avionics", True),

    # Cameras
    "/ROVER/camera_cs_0": ("rover", "Left", False),
    "/ROVER/camera_cs_1": ("rover", "UpLeft", False),
    "/ROVER/camera_cs_2": ("rover", "Right", False),
    "/ROVER/camera_cs_3": ("rover", "UpRight", False),
    "/ROVER/camera_cs_4": ("rover", "Other1", False),
    "/ROVER/camera_cs_5": ("rover", "Other2", False),
    
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
            if not self.model.rover_node.emergency_state:
                self.model.Nav.reset_informations()

        if "SC_motor_cmds" not in node_list:
            self.json['rover']['status']['systems']['drill']['status'] = 'Off'
            if not self.model.rover_node.emergency_state:
                self.model.Drill.reset_informations()
            
        if "HDCSInterfacing" not in node_list:
            self.json['rover']['status']['systems']['handling_device']['status'] = 'Off'
            if not self.model.rover_node.emergency_state:
                self.model.HD.reset_informations()
            
        if "python_node" not in node_list:
            if not self.model.rover_node.emergency_state:
                self.model.Elec.reset_informations()
        
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
                
            # if the node is not found
            else:
                if known_node_names[name][2]:
                    self.json['rover']['software']['nodes'][known_node_names[name][0]][known_node_names[name][1]]['status'] = False
                else:
                    self.json['cameras'][known_node_names[name][0]][known_node_names[name][1]]['node'] = False
