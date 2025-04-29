'''
=============== Node for interfacing between Rover and CS =================
Authors: Ugo Balducci, Giovanni Ranieri
Updated: 2024-2025

The main purpose of this node is to act as an orchestrator for the software of the rover.
The Rover node manages what is sent across the different subsystems. 
'''

import time, yaml
import rclpy
from rclpy.action import ActionServer, ActionClient

from std_msgs.msg       import String, Float32MultiArray, Float32, Bool
from std_srvs.srv       import SetBool
import sys

from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from custom_msg.msg import ServoRequest, ScMotorStatus, MotorStatus, ScFSMStatusDrill, OldMotorStatus # OldMotorStatus is for HD
from custom_msg.action import HDManipulation, DrillCmd, NAVReachGoal, NewHDGoal
from custom_msg.srv import ChangeModeSystem, DrillMode, ChangeModeCamera #ChangeModeHDCamera
from nav2_msgs.action import NavigateToPose

from bson import json_util
import json
from .model import NewModel
from .network_monitoring import NetworkMonitoring
from .active_node_checker import ActiveNodeChecker

class RoverNode():

    def __init__(self):

        rclpy.init(args=sys.argv)
        self.node = rclpy.create_node("ROVER")

        # Load the config files of the custom messages
        with open("/home/xplore/dev_ws/src/rover_pkg/rover_pkg/template_state.json") as json_file:
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

        # Parameters Launch file
        self.node.declare_parameter("network_node", False)
        self.network_node = self.node.get_parameter("network_node").get_parameter_value().bool_value

        # Create the models
        self.model = NewModel(self)

        # Potential Network Node (will be instancied only if necessary)
        self.network_monitor = None

        # group ros
        reentrant_callback_group = ReentrantCallbackGroup()

        # publisher of the rover state with the timer
        self.rover_state_pub = self.node.create_publisher(String, 
                                                          self.rover_names["rover_pubsub_state"], 1)
        self.timer = self.node.create_timer(0.1, self.timer_callback)

        # ==========================================================
        #              PUBLISHERS and SUBSCRIBERS
        # ==========================================================

        # -- NAV messages --
        self.nav_cmd_pub = self.node.create_publisher(Joy, self.rover_names["rover_pubsub_nav_gamepad"], 1)
        self.node.create_subscription(Joy, self.cs_names["cs_pubsub_nav_reachgoal"], self.transfer_gamepad_cmd_nav, 10)
        self.node.create_subscription(Odometry,         '/odometry/filtered',                self.model.Nav.nav_odometry  , 10)
        self.node.create_subscription(MotorStatus,    self.nav_names['nav_motors_status'],  self.model.Nav.nav_wheel, 10)
        self.node.create_subscription(Float32,    self.cs_names['cs_pubsub_speed_rover'],  self.model.Nav.change_speed_rover, 10)
        self.speed_rover_pub = self.node.create_publisher(Float32, self.rover_names["rover_change_nav_speed"], 1)

        # -- HD messages --
        self.hd_cmd_inverse_pub = self.node.create_publisher(Float32MultiArray, 
                                                             self.rover_names["rover_hd_man_inv_topic"], 1)
        self.hd_cmd_direct_pub = self.node.create_publisher(Float32MultiArray, 
                                                            self.rover_names["rover_hd_man_dir_topic"], 1)
        self.node.create_subscription(Joy, self.cs_names["cs_pubsub_hd_gamepad"], self.transfer_gamepad_cmd_hd, 10)
        self.node.create_subscription(
            OldMotorStatus, self.hd_names["hd_old_motor_status"], self.model.HD.hd_motor_cmds, 10)        


        # -- SC messages --
        self.node.create_subscription(ScMotorStatus, 
                                      self.science_names["science_pubsub_motor_status"], self.model.Drill.update_motor_status, 10)
        self.node.create_subscription(ScFSMStatusDrill, 
                                      self.science_names["science_pubsub_fms_status"], self.model.Drill.update_drill_status, 10)
        
        # -- Others --
        self.last_increment = 0
        self.cam_cmd_pub = self.node.create_publisher(ServoRequest, self.el_names["SERVO_REQ_TOPIC"], 1)

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
        
        # client to activate the rgbd mode of the HD camera
        self.change_camera_HD_mode_client = self.node.create_client(SetBool, 
                                                          '/ROVER/depth_req_camera_hd_0', callback_group=MutuallyExclusiveCallbackGroup())
        
        # The 7 next clients are to activate cameras
        self.camera_cs_service_0 = self.node.create_client(SetBool, 
                                                      '/ROVER/req_camera_cs_0', callback_group=MutuallyExclusiveCallbackGroup())

        self.camera_cs_service_1 = self.node.create_client(SetBool, 
                                                      '/ROVER/req_camera_cs_1', callback_group=MutuallyExclusiveCallbackGroup())
    
        self.camera_cs_service_2 = self.node.create_client(SetBool, 
                                                      '/ROVER/req_camera_cs_2', callback_group=MutuallyExclusiveCallbackGroup())
    
        self.camera_nav_service_0 = self.node.create_client(SetBool, 
                                                      '/NAV/req_camera_nav_0', callback_group=MutuallyExclusiveCallbackGroup())

        self.camera_nav_service_1 = self.node.create_client(SetBool, 
                                                      '/NAV/req_camera_nav_1', callback_group=MutuallyExclusiveCallbackGroup())
    
        self.camera_nav_service_2 = self.node.create_client(SetBool, 
                                                      '/NAV/req_camera_nav_2', callback_group=MutuallyExclusiveCallbackGroup())
                
        self.camera_hd_service_0 = self.node.create_client(SetBool, 
                                                      '/ROVER/req_camera_hd_0', callback_group=MutuallyExclusiveCallbackGroup())

        # The 7 next subscriber are to monitor the state of the cameras
        self.node.create_subscription(Bool, '/ROVER/state_camera_cs_0', self.model.cs_states_0, 10)
        
        self.node.create_subscription(Bool, '/ROVER/state_camera_cs_1', self.model.cs_states_1, 10)
        
        self.node.create_subscription(Bool, '/ROVER/state_camera_cs_2', self.model.cs_states_2, 10)
        
        self.node.create_subscription(Bool, '/NAV/state_camera_nav_0', self.model.nav_states_0, 10)
        
        self.node.create_subscription(Bool, '/NAV/state_camera_nav_1', self.model.nav_states_1, 10)
        
        self.node.create_subscription(Bool, '/NAV/state_camera_nav_2', self.model.nav_states_2, 10)
        
        self.node.create_subscription(Bool, '/HD/state_camera_hd_0', self.model.hd_states_0, 10)

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
        
        # The 3 next clients forward the action to the subsystem 
        # exception with navigation because it's nav2 that handles the action
        self.hd_action_client = ActionClient(self.node, NewHDGoal, self.rover_names["rover_hd_action_manipulation"])

        #self.nav_action_client = ActionClient(self.node, NavigateToPose, self.rover_names["rover_action_nav_goal"])

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
        msg = String()
        self.rover_state_json["timestamp"] = int(time.time()) # epoch

        msg.data = json.dumps(self.rover_state_json)
        self.clear_rover_msgs()
        self.rover_state_pub.publish(msg)

    def clear_rover_msgs(self):
        self.rover_state_json['rover']['status']['errors'] = []
        self.rover_state_json['rover']['status']['warnings'] = []

    # Transfer the gamepad commands for navigation
    def transfer_gamepad_cmd_nav(self, msg):
        
        # Transfer the camera commands
        self.transfer_gamepad_cmd_camera(msg)

        # Button 1 => change subsystem mode

        state = self.rover_state_json['rover']['status']['systems']['navigation']['status']

        # For safety, we change nothing if the state is auto and we click on 
        # changing the mode. 
        if (msg.buttons[1] == 1 and state == 'Auto'):
            return

        # Change to Ackermann
        if (msg.buttons[1] == 1 and state == 'Omni'):
            req = ChangeModeSystem.Request()
            req.system = 0
            req.mode = 1

            future = self.nav_service.call_async(req)
            future.add_done_callback(lambda f: self.model.Nav.service_callback_nav(f, req.mode))
        
        # Change to Omni
        elif (msg.buttons[1] == 1 and state == 'Ackermann'):
            req = ChangeModeSystem.Request()
            req.system = 0
            req.mode = 2

            future = self.nav_service.call_async(req)
            future.add_done_callback(lambda f: self.model.Nav.service_callback_nav(f, req.mode))

        self.nav_cmd_pub.publish(msg)

    # Transfer the gamepad commands for the arm
    def transfer_gamepad_cmd_hd(self, msg):
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
        increase = msg.buttons[2]
        decrease = msg.buttons[3]
        
        if increase == 0 and decrease == 0:
            self.last_increment = 0
        
        if self.last_increment == 1 and increase == 1: return
        if self.last_increment == 1 and decrease == -1: return
        angle = ServoRequest()
        angle.id = 1 
        angle.zero_in = False
        if increase == 1:
            angle.increment = 13
            self.cam_cmd_pub.publish(angle)
            self.last_increment = 1
        elif decrease == -1:
            angle.increment = -13
            self.cam_cmd_pub.publish(angle)
            self.last_increment = 1
                

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
