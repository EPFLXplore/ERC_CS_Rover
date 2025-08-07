import time, yaml
import rclpy
from rclpy.action import ActionServer, ActionClient

from std_msgs.msg       import String, Float32MultiArray, Float32, Bool
from std_srvs.srv       import SetBool
import sys

from sensor_msgs.msg import Joy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy


from custom_msg.msg import ServoRequest, ScMotorStatus, MotorStatus, ScFSMStatusDrill, OldMotorStatus # OldMotorStatus is for HD
from custom_msg.action import HDManipulation, DrillCmd, NAVReachGoal, NewHDGoal
from custom_msg.srv import ChangeModeSystem, DrillMode, ChangeModeCamera #ChangeModeHDCamera
from nav2_msgs.action import NavigateToPose

import json
from bson import json_util
from .model import NewModel
from .network_monitoring import NetworkMonitoring
from .active_node_checker import ActiveNodeChecker

'''
=============== Node for interfacing between Rover and CS =================
Authors: Ugo Balducci & Giovanni Ranieri
Updated: 2023-2025

The main purpose of this node is to act as an orchestrator for the software of the rover.
The Rover node manages what is sent across the different subsystems. 
'''

class RoverNode():

    def __init__(self):

        rclpy.init(args=sys.argv)
        self.node = rclpy.create_node("ROVER")

        # Load the config files of the custom messages
        with open("/home/xplore/dev_ws/src/rover_pkg/rover_pkg/rover_state.json") as json_file:
            self.rover_state_json = dict(json.load(json_file))
        
        with open('/home/xplore/dev_ws/src/custom_msg/config/cs_interface_names.yaml', 'r') as file:
            self.cs_names = yaml.safe_load(file)["/**"]["ros__parameters"]
            
        with open('/home/xplore/dev_ws/src/custom_msg/config/hd_interface_names.yaml', 'r') as file:
            self.hd_names = yaml.safe_load(file)["/**"]["ros__parameters"]
        
        with open('/home/xplore/dev_ws/src/custom_msg/config/rover_interface_names.yaml', 'r') as file:
            self.rover_names = yaml.safe_load(file)["/**"]["ros__parameters"]
            
        with open('/home/xplore/dev_ws/src/custom_msg/config/science_interface_names.yaml', 'r') as file:
            self.science_names = yaml.safe_load(file)["/**"]["ros__parameters"]

        with open('/home/xplore/dev_ws/src/custom_msg/config/el_interface_names.yaml', 'r') as file:
            self.el_names = yaml.safe_load(file)["/**"]["ros__parameters"]

        with open('/home/xplore/dev_ws/src/custom_msg/config/nav_interface_names.yaml', 'r') as file:
            self.nav_names = yaml.safe_load(file)["/**"]["ros__parameters"]

        # Utility profile for some subscribers 
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT, # BEST_EFFORT: message will attempt to send message but if it fails it will not try again
            durability=QoSDurabilityPolicy.VOLATILE, # VOLATILE: if no subscribers are listening, the message sent is not saved
            history=QoSHistoryPolicy.KEEP_LAST, # KEEP_LAST: only the last n = depth messages are stored in the queue
            depth=10,
        )

        # Parameters Launch file
        self.node.declare_parameter("network_node", False)
        self.network_node = self.node.get_parameter("network_node").get_parameter_value().bool_value

        # Emergency state boolean: True means the CS requested an emergency
        self.emergency_state = False

        # Create the models
        self.model = NewModel(self)

        # Potential Network Node (will be instancied only if necessary)
        self.network_monitor = None

        # Group ROS
        reentrant_callback_group = ReentrantCallbackGroup()

        # Publisher of the rover state (2 Hz)
        self.rover_state_pub = self.node.create_publisher(String, 
                                                          self.rover_names["rover_pubsub_state"], 1)
        self.timer = self.node.create_timer(0.5, self.timer_callback)
        self.tmp_test = None

        # ==========================================================
        #              PUBLISHERS and SUBSCRIBERS
        # ==========================================================
        
        ## ------------ Cameras messages --------------
        
        self.node.create_subscription(Bool, self.cs_names["cs_take_screenshot_all_cameras"], self.screenshot_all_cameras, 10)

        ## ------------ NAV messages --------------
        
        # Forward the gamepad commands to navigation subsystem
        self.nav_cmd_pub = self.node.create_publisher(Joy, self.rover_names["rover_pubsub_nav_gamepad"], 1)
        
        # Listens to incoming gamepad commands from CS
        self.node.create_subscription(Joy, self.cs_names["cs_pubsub_nav_reachgoal"], self.transfer_gamepad_cmd_nav, 10)
        
        # Listens to navigation motor status
        self.node.create_subscription(MotorStatus,    self.nav_names['nav_motors_status'],  self.model.Nav.nav_wheel, qos_profile=self.qos_profile)
        
        # Listens to change of speed of motors from CS
        self.node.create_subscription(Float32,    self.cs_names['cs_pubsub_speed_rover'],  self.model.Nav.change_speed_rover, 10)
        
        # Forward the change of speed of motors from CS to navigation subsystem
        self.speed_rover_pub = self.node.create_publisher(Float32, self.rover_names["rover_change_nav_speed"], 1)
        
        # Listen to jetson stats from navigation subystem
        self.node.create_subscription(String, '/NAV/jetson_stats', self.jetson_stats_nav, 10)
        

        ## ------------ HDS messages --------------
        
        # Forward the gamepad commands to HD subsystem for inverse kinematics mode
        self.hd_cmd_inverse_pub = self.node.create_publisher(Float32MultiArray, 
                                                             self.rover_names["rover_hd_man_inv_topic"], 1)
        
        # Forward the gamepad commands to HD subsystem for direct kinematics mode
        self.hd_cmd_direct_pub = self.node.create_publisher(Float32MultiArray, 
                                                            self.rover_names["rover_hd_man_dir_topic"], 1)
        
        # Listen to jetson stats from HD subystem
        self.node.create_subscription(String, '/HD/jetson_stats', self.jetson_stats_hd, 10)
        
        # Listens to incoming gamepad commands from CS
        self.node.create_subscription(Joy, self.cs_names["cs_pubsub_hd_gamepad"], self.transfer_gamepad_cmd_hd, 10)
        
        # Listens to HD motor status
        self.node.create_subscription(
            OldMotorStatus, self.hd_names["hd_old_motor_status"], self.model.HD.hd_motor_cmds, 10)        


        ## ------------ Drill messages --------------
        
        # Listens to Drill motor status
        self.node.create_subscription(ScMotorStatus, 
                                      self.science_names["science_pubsub_motor_status"], self.model.Drill.update_motor_status, 10)
        
        # Listens to FSM status
        self.node.create_subscription(ScFSMStatusDrill, 
                                      self.science_names["science_pubsub_fms_status"], self.model.Drill.update_drill_status, 10)
        
        
        ## ------------ Elec messages --------------
        
        # Publisher for moving the front camera of navigation, directly from CS
        self.last_increment = 0
        self.cam_cmd_pub = self.node.create_publisher(ServoRequest, self.el_names["SERVO_REQ_TOPIC"], 1)
        self.switching_nav = False
        self.switching_hd = False
        self.last_hd_switch_time = None
        self.last_nav_switch_time = None

        # ==========================================================
        #                       SERVICES
        # ==========================================================

        # server to change mode of a subsystem
        self.change_rover_mode = self.node.create_service(ChangeModeSystem, 
                                                          self.cs_names["cs_service_change_subsystem"], self.model.change_mode_system_service, callback_group=MutuallyExclusiveCallbackGroup())

        # client to change mode of navigation
        self.nav_service = self.node.create_client(ChangeModeSystem, self.rover_names["rover_change_nav_mode"], callback_group=MutuallyExclusiveCallbackGroup())
        
        # client to change mode of handling device
        self.hd_mode_service = self.node.create_client(ChangeModeSystem, 
                                                       self.rover_names["rover_change_hd_mode"], callback_group=MutuallyExclusiveCallbackGroup())

        # client to change mode of drill device
        self.drill_service = self.node.create_client(DrillMode, 
                                                       self.science_names["drill_mode_srv"], callback_group=MutuallyExclusiveCallbackGroup())    

        # server to change mode of a camera
        self.change_camera_mode = self.node.create_service(ChangeModeCamera, 
                                                          self.cs_names["cs_change_mode_camera"], self.model.change_mode_camera_service, callback_group=MutuallyExclusiveCallbackGroup()) 
        
        # server to activate the rgbd mode of the HD camera
        self.change_camera_HD_mode = self.node.create_service(SetBool, 
                                                          self.cs_names["cs_change_mode_camera_HD"], self.model.change_mode_camera_HD_service, callback_group=MutuallyExclusiveCallbackGroup())
        
         # server to activate the rgbd mode of the NAV camera
        self.change_camera_NAV_mode = self.node.create_service(SetBool, 
                                                          self.cs_names["cs_change_mode_camera_NAV"], self.model.change_mode_camera_NAV_service, callback_group=MutuallyExclusiveCallbackGroup())
        
        # client to activate the rgbd mode of the HD camera
        self.change_camera_HD_mode_client = self.node.create_client(SetBool, 
                                                          '/ROVER/depth_req_camera_hd_0', callback_group=MutuallyExclusiveCallbackGroup())
        
        # client to activate the rgbd mode of the NAV camera
        self.change_camera_NAV_mode_client = self.node.create_client(SetBool, 
                                                          '/NAV/depth_req_camera_nav_0', callback_group=MutuallyExclusiveCallbackGroup())
        
        # The 7 next clients are to activate cameras
        
        self.camera_cs_service_0 = self.node.create_client(SetBool, 
                                                      '/ROVER/req_camera_cs_0', callback_group=MutuallyExclusiveCallbackGroup())

        self.camera_cs_service_1 = self.node.create_client(SetBool, 
                                                      '/ROVER/req_camera_cs_1', callback_group=MutuallyExclusiveCallbackGroup())
    
        self.camera_cs_service_2 = self.node.create_client(SetBool, 
                                                      '/ROVER/req_camera_cs_2', callback_group=MutuallyExclusiveCallbackGroup())
        
        self.camera_cs_service_3 = self.node.create_client(SetBool, 
                                                      '/ROVER/req_camera_cs_3', callback_group=MutuallyExclusiveCallbackGroup())
        
        self.camera_cs_service_4 = self.node.create_client(SetBool, 
                                                      '/ROVER/req_camera_cs_4', callback_group=MutuallyExclusiveCallbackGroup())
        
        self.camera_cs_service_5 = self.node.create_client(SetBool, 
                                                      '/ROVER/req_camera_cs_5', callback_group=MutuallyExclusiveCallbackGroup())
    
        self.camera_nav_service_0 = self.node.create_client(SetBool, 
                                                      '/NAV/req_camera_nav_0', callback_group=MutuallyExclusiveCallbackGroup())

        self.camera_nav_service_1 = self.node.create_client(SetBool, 
                                                      '/NAV/req_camera_nav_1', callback_group=MutuallyExclusiveCallbackGroup())
    
        self.camera_nav_service_2 = self.node.create_client(SetBool, 
                                                      '/NAV/req_camera_nav_2', callback_group=MutuallyExclusiveCallbackGroup())
                
        self.camera_hd_service_0 = self.node.create_client(SetBool, 
                                                      '/ROVER/req_camera_hd_0', callback_group=MutuallyExclusiveCallbackGroup())
        
        self.node.create_subscription(Bool, '/ROVER/state_depth_camera_hd_0', self.model.hd_states_0, 10)
        
        # The 5 next clients are for screenshots during the Exploration Task
        
        self.camera_cs_screenshot_0 = self.node.create_client(SetBool, 
                                                      '/ROVER/screenshot_camera_cs_0', callback_group=MutuallyExclusiveCallbackGroup())

        self.camera_cs_screenshot_1 = self.node.create_client(SetBool, 
                                                      '/ROVER/screenshot_camera_cs_1', callback_group=MutuallyExclusiveCallbackGroup())
    
        self.camera_cs_screenshot_2 = self.node.create_client(SetBool, 
                                                      '/ROVER/screenshot_camera_cs_2', callback_group=MutuallyExclusiveCallbackGroup())
        
        self.camera_cs_screenshot_3 = self.node.create_client(SetBool, 
                                                      '/ROVER/screenshot_camera_cs_3', callback_group=MutuallyExclusiveCallbackGroup()) 
        
        self.camera_nav_screenshot_0 = self.node.create_client(SetBool, 
                                                      '/NAV/screenshot_camera_nav_0', callback_group=MutuallyExclusiveCallbackGroup())


        # ==========================================================
        #                       ACTIONS
        # ==========================================================

        # server that handle CS request for a manipulation task
        self.hd_manipulation_action = ActionServer(self.node, HDManipulation, 
                                                   self.cs_names["cs_hd_action_manipulation"], execute_callback=self.model.HD.make_action,
                                                callback_group=reentrant_callback_group,
                                                goal_callback=self.model.HD.action_status, cancel_callback=self.model.HD.cancel_goal_from_cs)

        # server that handle CS request for a autonomous task in navigation                                   
        self.nav_reach_goal_action = ActionServer(self.node, NAVReachGoal, 
                                                  self.cs_names["cs_action_nav_goal"], execute_callback=self.model.Nav.make_action,
                                                  callback_group=reentrant_callback_group,
                                                  goal_callback=self.model.Nav.action_status, cancel_callback=self.model.Nav.cancel_goal)

        # server that handle CS request for a drill task
        self.drill_action = ActionServer(self.node, DrillCmd, 
                                          self.cs_names["cs_action_drill"], execute_callback=self.model.Drill.make_action, 
                                          callback_group=reentrant_callback_group,
                                          goal_callback=self.model.Drill.action_status, cancel_callback=self.model.Drill.cancel_goal_from_cs)
        
        # The 2 next clients forward the action to the subsystem 
        
        self.hd_action_client = ActionClient(self.node, NewHDGoal, self.rover_names["rover_hd_action_manipulation"])

        self.drill_action_client = ActionClient(self.node, DrillCmd, self.rover_names['rover_action_drill_state'])


        # ==========================================================
        #                       LAUNCHING
        # ==========================================================


        self.node.get_logger().info("Rover Node Started")
        
        if self.network_node:
            self.network_monitor = NetworkMonitoring(rover_state=self.rover_state_json,
                                                     node=self.node)
        else:
            self.node.get_logger().info("No Networking Node")

        # Start the health node checker
        self.health = ActiveNodeChecker(self.rover_state_json, self.model)
            
    # timer callback for sending rover state continuously
    def timer_callback(self):
        current_time = time.time()
        msg = String()
        msg.data = json.dumps(self.rover_state_json)
        self.rover_state_pub.publish(msg)
        if self.tmp_test is None or (current_time - self.tmp_test) > 3:
            self.node.get_logger().info("publisher rover node dead or init node")
            self.tmp_test = current_time
        
        self.tmp_test = time.time()

    # Transfer the gamepad commands for navigation
    def transfer_gamepad_cmd_nav(self, msg):
        
        # Security check is the message is really for navigation
        if int(msg.buttons[0]) != 1: return
        
        # For switching mode
        switch = int(msg.buttons[1])   
        
        # For switching mode
        switch = int(msg.buttons[1])
        current_time = time.time()
        if switch == 1:
            if not self.switching_nav:
                if self.last_nav_switch_time is None or (current_time - self.last_nav_switch_time) > 3:
                    self.switching_hd = True
                    self.last_nav_switch_time = current_time
                    # switch function
                    new_mode = 1 if self.rover_state_json['rover']['status']['systems']['navigation']['status'] == "Omni" else 2
                    self.model.send_nav_service(0, new_mode)
            else:
                # Already switching, ignore
                pass
        else:
            self.switching_nav = False
        
        # Transfer the camera commands
        self.transfer_gamepad_cmd_camera(msg)

        self.nav_cmd_pub.publish(msg)

    # Transfer the gamepad commands for the arm
    def transfer_gamepad_cmd_hd(self, msg):
        
        # Security check is the message is really for handling device
        if int(msg.buttons[0]) != 2: return
        
        # For switching mode
        switch = int(msg.buttons[1])
        current_time = time.time()
        if switch == 1:
            if not self.switching_hd:
                if self.last_hd_switch_time is None or (current_time - self.last_hd_switch_time) > 3:
                    self.switching_hd = True
                    self.last_hd_switch_time = current_time
                    # switch function
                    new_mode = 1 if self.rover_state_json['rover']['status']['systems']['handling_device']['status'] == "Manual Inverse" else 2
                    self.model.send_hd_service(1, new_mode)
            else:
                # Already switching, ignore
                pass
        else:
            self.switching_hd = False
        
        if(self.rover_state_json['rover']['status']['systems']['handling_device']['status'] == "Manual Direct"):
            msgHD = Float32MultiArray()
            msgHD.data = msg.axes
            self.hd_cmd_direct_pub.publish(msgHD)

        if(self.rover_state_json['rover']['status']['systems']['handling_device']['status'] == "Manual Inverse"):
            msgHD = Float32MultiArray()
            msgHD.data = msg.axes
            self.hd_cmd_inverse_pub.publish(msgHD)
        
    # Transfer the gamepad commands for the navigation camera
    def transfer_gamepad_cmd_camera(self, msg):
        increase = msg.buttons[2] # +1
        decrease = msg.buttons[3] # -1
        
        if increase == 0 and decrease == 0:
            self.last_increment = 0
        
        if self.last_increment == 1 and increase == 1: return
        if self.last_increment == 1 and decrease == -1: return
        angle = ServoRequest()
        angle.id = 1 
        angle.zero_in = False
        if increase == 1:
            angle.increment = 20
            self.cam_cmd_pub.publish(angle)
            self.last_increment = 1
        elif decrease == -1:
            angle.increment = -20
            self.cam_cmd_pub.publish(angle)
            self.last_increment = 1
            
    def screenshot_all_cameras(self, msg):
        if msg.data:
            # Take a screenshot of all cameras
            self.camera_cs_screenshot_0.call_async(SetBool.Request(data=True))
            self.camera_cs_screenshot_1.call_async(SetBool.Request(data=True))
            self.camera_cs_screenshot_2.call_async(SetBool.Request(data=True))
            self.camera_cs_screenshot_3.call_async(SetBool.Request(data=True))
            self.camera_nav_screenshot_0.call_async(SetBool.Request(data=True))
            
    
    def jetson_stats_hd(self, msg):
        self.rover_state_json['rover']['hardware']['stats_hd'] = json.loads(msg.data)
        
    def jetson_stats_nav(self, msg):
        self.rover_state_json['rover']['hardware']['stats_nav'] = json.loads(msg.data)
                

    def run(self):
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self.node)
        if self.network_monitor != None:
            executor.add_node(self.network_monitor)
        
        executor.add_node(self.health)
        executor.spin()
        rclpy.shutdown()

def main():
    rover = RoverNode()
    rover.run()
