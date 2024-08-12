# ===============Node for interfacing between Rover and CS=================

import time, yaml
import rclpy
from rclpy.action import ActionServer
#from config import NAME_ROVER_NODE, NAME_ROVER_2025, TEMPLATE_STATE_PATH, INTERFACE_NAMES_PATH

from std_msgs.msg       import Int8, String, Float32MultiArray
from std_srvs.srv       import SetBool
import sys

# from geometry_msgs.msg  import Twist, PoseStamped
# from actionlib_msgs.msg import GoalID
from sensor_msgs.msg import JointState, Joy
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup

from custom_msg.msg import Wheelstatus, Motorcmds, MassArray, ScMotorStatus
from custom_msg.action import HDManipulation, NAVReachGoal, DrillTerrain, DrillCmd
from custom_msg.srv import ChangeModeSystem, HDMode

from rover_pkg.db_logger import MongoDBLogger
from bson import json_util

# from std_srvs.srv import SetBool
import json, threading
from .new_model import NewModel


class RoverNode():
    """Aggregates the rover topics into single JSON and sends it 
    to the CS, and conversely receives commands from the CS and 
    publishes them to the rover topics"""

    def __init__(self):

        self.t1 = time.time()
        rclpy.init(args=sys.argv)
        self.node = rclpy.create_node("ROVER")

        self.model = NewModel(self)
        self.logger = MongoDBLogger("Onyx", "rover_state")

        with open("/home/xplore/dev_ws/src/rover_pkg/rover_pkg/template_state.json") as json_file:
            self.rover_state_json = dict(json.load(json_file))
        
        with open('/home/xplore/dev_ws/src/custom_msg/config/cs_interface_names.yaml', 'r') as file:
            self.cs_names = yaml.safe_load(file)
            
        with open('/home/xplore/dev_ws/src/custom_msg/config/hd_interface_names.yaml', 'r') as file:
            self.hd_names = yaml.safe_load(file)
        
        with open('/home/xplore/dev_ws/src/custom_msg/config/rover_interface_names.yaml', 'r') as file:
            self.rover_names = yaml.safe_load(file)
            
        with open('/home/xplore/dev_ws/src/custom_msg/config/science_interface_names.yaml', 'r') as file:
            self.science_names = yaml.safe_load(file)

        # ==========================================================
        #              MESSAGES BETWEEN ROVER AND CS
        # ==========================================================

        reentrant_callback_group = ReentrantCallbackGroup()

        # ===== PUBLISHERS =====

        self.rover_state_pub = self.node.create_publisher(String, 
                                                          self.rover_names["ros__parameters"]["rover_pubsub_state"], 1)
        self.timer = self.node.create_timer(0.1, self.timer_callback)

        # -- NAV messages --
        self.nav_cmd_pub = self.node.create_publisher(Joy, 
                                                      self.cs_names["ros__paramters"]["cs_pubsub_nav_gamepad"], 1) # Name to be changed
        self.nav_mode_pub = self.node.create_publisher(String, 
                                                       self.rover_names["ros__parameters"]["rover_pubsub_nav_mode"], 1)

        # -- HD messages --
        self.hd_cmd_inverse_pub = self.node.create_publisher(Float32MultiArray, 
                                                             self.rover_names["ros__parameters"]["rover_hd_man_inv_topic"], 1)
        self.hd_cmd_direct_pub = self.node.create_publisher(Float32MultiArray, 
                                                            self.rover_names["ros__parameters"]["rover_hd_man_dir_topic"], 1)
        self.hd_mode_pub = self.node.create_publisher(Int8, 
                                                      self.rover_names["ros__parameters"]["rover_hd_mode"], 10)

        # -- SC messages --
        self.sc_cmd_pub = self.node.create_publisher(String, 
                                                     self.science_names["ros__parameters"]["science_pubsub_drill"], 1)
        
        # ===== SUBSCRIBERS =====
        
        self.node.create_subscription(Joy, self.cs_names["ros__parameters"]["cs_action_nav_reachgoal"], self.transfer_gamepad_cmd_nav, 10)
        self.node.create_subscription(Joy, self.cs_names["ros__parameters"]["cs_pubsub_hd_gamepad"], self.transfer_gamepad_cmd_hd, 10)

        self.node.create_subscription(String, self.rover_names["ros__parameters"]["rover_pubsub_perf"], self.model.update_metrics, 10)

        # -- SC messages --
        self.node.create_subscription(ScMotorStatus, 
                                      self.science_names["ros__parameters"]["science_pubsub_motor_status"], self.model.Drill.update_motor_status, 10)
        self.node.create_subscription(String, 
                                      self.science_names["ros__parameters"]["science_pubsub_fms_status"], self.model.Drill.update_drill_status, 10)
      
        self.node.create_subscription(MassArray, 
                                      self.science_names["ros__parameters"]["science_pubsub_drill_mass"], self.model.Elec.update_mass_measurement, 10)


        # -- HD messages --
        self.node.create_subscription(
            JointState, self.hd_names["ros__parameters"]["hd_motor_telemetry"], self.model.HD.hd_joint_state, 10)

        # -- NAV messages --
        self.node.create_subscription(Odometry,         '/lio_sam/odom',                self.model.Nav.nav_odometry  , 10)
        self.node.create_subscription(Wheelstatus,      '/NAV/absolute_encoders',       self.model.Nav.nav_wheel, 10)
        self.node.create_subscription(Motorcmds,        '/NAV/displacement',            self.model.Nav.nav_displacement, 10)

        # ===== SERVICES =====

        self.change_rover_mode = self.node.create_service(ChangeModeSystem, 
                                                          self.rover_names["ros__parameters"]["rover_service_change_subsystem"], self.model.change_mode_system_service, callback_group=reentrant_callback_group)

        self.camera_service = self.node.create_client(SetBool, 
                                                      self.rover_names["ros__parameters"]["rover_service_cameras_start"], callback_group=reentrant_callback_group)
                
        self.hd_mode_service = self.node.create_client(HDMode, 
                                                       self.hd_names["ros__parameters"]["hd_fsm_mode_srv"], callback_group=reentrant_callback_group)

        # ===== ACTIONS =====

        self.hd_manipulation_action = ActionServer(self.node, HDManipulation, 
                                                   self.rover_names["ros__parameters"]["rover_hd_action_manipulation"], self.model.HD.make_action,
                                                goal_callback=self.model.HD.action_status)

        self.nav_reach_goal_action = ActionServer(self.node, NAVReachGoal, 
                                                  self.rover_names["ros__parameters"]["rover_action_nav_goal"], self.model.Nav.make_action,
                                                  goal_callback=self.model.Nav.action_status)

        self.drill_action = ActionServer(self.node, DrillCmd, 
                                          self.rover_names["ros__parameters"]["rover_action_drill"], self.model.Drill.make_action, 
                                          goal_callback=self.model.Drill.action_status)

        # -- CS messages -

        self.node.get_logger().info("Rover Node Started")

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
        self.rover_state_json['rover']['status']['error'] = []
        self.rover_state_json['rover']['status']['warning'] = []

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
        executor.spin()
        rclpy.shutdown()



def main():
    rover = RoverNode()
    rover.run()
