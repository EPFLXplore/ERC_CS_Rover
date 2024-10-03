'''
=============== Node for interfacing between Rover and CS =================
Authors: Ugo Balducci, Giovanni Ranieri
Updated: 2024

The main purpose of this class is to act as an orchestrator for the software of the rover.
The Rover node manages what is sent using ROS across the different subsystems. 
'''

import time, yaml
import rclpy
from rclpy.action import ActionServer, ActionClient

from std_msgs.msg       import String, Float32MultiArray
from std_srvs.srv       import SetBool
import sys

from sensor_msgs.msg import JointState, Joy
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from custom_msg.msg import Wheelstatus, Motorcmds, ScMotorStatus
from custom_msg.action import HDManipulation, DrillCmd, NAVReachGoal
from custom_msg.srv import ChangeModeSystem, HDMode, DrillMode, RequestHDGoal
from nav2_msgs.action import NavigateToPose


from rover_pkg.db_logger import MongoDBLogger
from bson import json_util
import json
from .new_model import NewModel
from .network_monitoring import NetworkMonitoring


class RoverNode():
    """Aggregates the rover topics into single JSON and sends it 
    to the CS, and conversely receives commands from the CS and 
    publishes them to the rover topics"""

    def __init__(self):

        self.t1 = time.time()
        rclpy.init(args=sys.argv)
        self.node = rclpy.create_node("ROVER")

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

        self.model = NewModel(self)
        self.logger = MongoDBLogger("Onyx", "rover_state")
        self.network_monitor = None

        # ==========================================================
        #              MESSAGES BETWEEN ROVER AND CS
        # ==========================================================

        reentrant_callback_group = ReentrantCallbackGroup()

        # ===== PUBLISHERS =====

        self.rover_state_pub = self.node.create_publisher(String, 
                                                          self.rover_names["rover_pubsub_state"], 1)
        self.timer = self.node.create_timer(0.1, self.timer_callback)

        # -- NAV messages --
        self.nav_cmd_pub = self.node.create_publisher(Joy, self.cs_names["cs_pubsub_nav_gamepad"], 1)

        # WILL BE CONVERTED TO SERVICE
        self.nav_mode_pub = self.node.create_publisher(String, 
                                                       self.rover_names["rover_pubsub_nav_mode"], 1)

        # -- HD messages --
        self.hd_cmd_inverse_pub = self.node.create_publisher(Float32MultiArray, 
                                                             self.rover_names["rover_hd_man_inv_topic"], 1)
        self.hd_cmd_direct_pub = self.node.create_publisher(Float32MultiArray, 
                                                            self.rover_names["rover_hd_man_dir_topic"], 1)
        
        # ===== SUBSCRIBERS =====
        
        self.node.create_subscription(Joy, self.cs_names["cs_action_nav_reachgoal"], self.transfer_gamepad_cmd_nav, 10)
        
        self.node.create_subscription(Joy, self.cs_names["cs_pubsub_hd_gamepad"], self.transfer_gamepad_cmd_hd, 10)

        # -- SC messages --
        self.node.create_subscription(ScMotorStatus, 
                                      self.science_names["science_pubsub_motor_status"], self.model.Drill.update_motor_status, 10)
        self.node.create_subscription(String, 
                                      self.science_names["science_pubsub_fms_status"], self.model.Drill.update_drill_status, 10)
      
        # -- HD messages --
        self.node.create_subscription(
            JointState, self.hd_names["hd_motor_telemetry"], self.model.HD.hd_joint_state, 10)

        # -- NAV messages --
        self.node.create_subscription(Odometry,         '/lio_sam/odom',                self.model.Nav.nav_odometry  , 10)
        self.node.create_subscription(Wheelstatus,      '/NAV/absolute_encoders',       self.model.Nav.nav_wheel, 10)
        self.node.create_subscription(Motorcmds,        '/NAV/displacement',            self.model.Nav.nav_displacement, 10)

        # ===== SERVICES =====

        self.change_rover_mode = self.node.create_service(ChangeModeSystem, 
                                                          self.cs_names["cs_service_change_subsystem"], self.model.change_mode_system_service, callback_group=MutuallyExclusiveCallbackGroup())

        self.nav_service = self.node.create_client(ChangeModeSystem, '/ROVER/change_NAV_mode', callback_group=MutuallyExclusiveCallbackGroup())

        self.camera_service = self.node.create_client(SetBool, 
                                                      self.rover_names["rover_service_cameras_start"], callback_group=MutuallyExclusiveCallbackGroup())
                
        self.hd_mode_service = self.node.create_client(HDMode, 
                                                       self.hd_names["hd_fsm_mode_srv"], callback_group=MutuallyExclusiveCallbackGroup())

        self.drill_service = self.node.create_client(DrillMode, 
                                                       self.science_names["drill_mode_srv"], callback_group=MutuallyExclusiveCallbackGroup())


        # ===== ACTIONS + SERVICES =====

        self.hd_manipulation_action = ActionServer(self.node, HDManipulation, 
                                                   self.cs_names["cs_hd_action_manipulation"], execute_callback=self.model.HD.make_action,
                                                
                                                goal_callback=self.model.HD.action_status)

        self.hd_manipulation_service = self.node.create_client(RequestHDGoal, 
                                                   self.hd_names["hd_fsm_goal_srv"], callback_group=MutuallyExclusiveCallbackGroup())
                                                
        self.nav_reach_goal_action = ActionServer(self.node, NAVReachGoal, 
                                                  self.cs_names["cs_action_nav_goal"], self.model.Nav.make_action,
                                                  goal_callback=self.model.Nav.action_status, cancel_callback=self.model.Nav.cancel_goal)

        self.drill_action = ActionServer(self.node, DrillCmd, 
                                          self.cs_names["cs_action_drill"], execute_callback=self.model.Drill.make_action, 
                                          callback_group=reentrant_callback_group,
                                          goal_callback=self.model.Drill.action_status, cancel_callback=self.model.Drill.cancel_goal_from_cs)
        
        self.hd_action_client = ActionClient(self.node, HDManipulation, self.rover_names["rover_hd_action_manipulation"])

        #self.nav_action_client = ActionClient(self.node, NavigateToPose, self.rover_names["rover_action_nav_goal"])

        self.drill_action_client = ActionClient(self.node, DrillCmd, self.rover_names['rover_action_drill_state'])


        self.node.get_logger().info("Rover Node Started")
        
        if len(sys.argv) > 1 and sys.argv[1] == 'true':
            self.network_monitor = NetworkMonitoring(rover_state=self.rover_state_json)
        else:
            self.node.get_logger().info("No Networking")
            
    # timer callback for sending rover state continuously
    def timer_callback(self):
        msg = String()
        self.rover_state_json["timestamp"] = int(time.time()) # epoch

        # Log in MongoDB
        rover_bson = json.loads(json_util.dumps(self.rover_state_json))
        self.logger.log(rover_bson)

        msg.data = json.dumps(self.rover_state_json)
        self.clear_rover_msgs()
        self.rover_state_pub.publish(msg)

    def clear_rover_msgs(self):
        self.rover_state_json['rover']['status']['errors'] = []
        self.rover_state_json['rover']['status']['warnings'] = []

    def transfer_gamepad_cmd_nav(self, msg):
        # TODO: Check that the mode is set to MANUAL
        self.nav_cmd_pub.publish(msg)

    def transfer_gamepad_cmd_hd(self, msg):
        if(self.rover_state_json['rover']['status']['systems']['handling_device']['status'] == "Manual Direct"):
            msgHD = Float32MultiArray()
            msgHD.data = msg.axes
            self.hd_cmd_direct_pub.publish(msgHD)

        if(self.rover_state_json['rover']['status']['systems']['handling_device']['status'] == "Manual Inverse"):
            msgHD = Float32MultiArray()
            msgHD.data = msg.axes
            self.hd_cmd_inverse_pub.publish(msgHD)
        

    def run(self):
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self.node)
        if self.network_monitor != None:
            executor.add_node(self.network_monitor)
        executor.spin()
        rclpy.shutdown()



def main():
    rover = RoverNode()
    rover.run()