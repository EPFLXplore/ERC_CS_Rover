# ===============Node for interfacing between Rover and CS=================

import time
import rclpy
from rclpy.logging import LoggingSeverity
from rclpy.action import ActionServer
# import math
import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from ...config import NAME_ROVER_NODE, NAME_ROVER_2025, TEMPLATE_STATE_PATH, INTERFACE_NAMES_PATH

from std_msgs.msg       import Int8, Int16, Int32, Bool, String, Int8MultiArray,  Int16MultiArray, Float32MultiArray, UInt8MultiArray
from std_srvs.srv       import SetBool

# from geometry_msgs.msg  import Twist, PoseStamped
# from actionlib_msgs.msg import GoalID
from sensor_msgs.msg import JointState, Joy
from nav_msgs.msg import Odometry, Path
from rclpy.callback_groups import ReentrantCallbackGroup
# from diagnostic_msgs.msg import DiagnosticStatus

from custom_msg.msg import Wheelstatus, Motorcmds, MassArray, ScMotorStatus
from custom_msg.action import HDManipulation, NAVReachGoal, DrillTerrain, DrillCmd
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from custom_msg.srv import ChangeModeSystem, HDMode

from rover_pkg.db_logger import MongoDBLogger
from bson import json_util

# from std_srvs.srv import SetBool
import json

import threading


from .new_model import NewModel
# from .launcher import *


class RoverNode():
    """Aggregates the rover topics into single JSON and sends it 
    to the CS, and conversely receives commands from the CS and 
    publishes them to the rover topics"""

    def __init__(self):

        self.t1 = time.time()
        rclpy.init(args=sys.argv)
        self.node = rclpy.create_node(NAME_ROVER_NODE)

        self.model = NewModel(self)
        self.logger = MongoDBLogger(NAME_ROVER_2025, "rover_state")

        with open(TEMPLATE_STATE_PATH) as json_file:
            self.rover_state_json = dict(json.load(json_file))
        
        with open(INTERFACE_NAMES_PATH) as file:
            cs_names = json.load(file)

        # ==========================================================
        #              MESSAGES BETWEEN ROVER AND CS
        # ==========================================================

        reentrant_callback_group = ReentrantCallbackGroup()

        # ===== PUBLISHERS =====

        self.rover_state_pub = self.node.create_publisher(String, cs_names["CS"]["rover_state"], 1)
        self.timer = self.node.create_timer(0.1, self.timer_callback)

        # -- NAV messages --
        self.nav_cmd_pub = self.node.create_publisher(Joy, '/CS/NAV_gamepad', 1) # Name to be changed
        self.nav_mode_pub = self.node.create_publisher(String, '/ROVER/NAV_mode', 1)

        # -- HD messages --
        # self.hd_cmd_pub = self.node.create_publisher(Float32MultiArray, "/CS/HD_gamepad", 10)
        self.hd_cmd_inverse_pub = self.node.create_publisher(Float32MultiArray, "/ROVER/HD/manual_inverse", 1)
        self.hd_cmd_direct_pub = self.node.create_publisher(Float32MultiArray, "/ROVER/HD/manual_direct", 1)
        # self.man_inv_axis_pub = self.create_publisher(Float32MultiArray, "/ROVER/HD_man_inv_axis", 10)
        self.hd_mode_pub = self.node.create_publisher(Int8, "/ROVER/HD_mode", 10)

        # -- SC messages --
        self.sc_cmd_pub = self.node.create_publisher(String, "/SC/drill_cmd", 1)

        # ===== SERVICES =====

        self.change_rover_mode = self.node.create_service(ChangeModeSystem, cs_names["CS"]["change_system"], self.model.change_mode_system_service, callback_group=reentrant_callback_group)

        self.camera_service = self.node.create_client(SetBool, '/ROVER/start_cameras', callback_group=reentrant_callback_group)
                
        self.hd_mode_service = self.node.create_client(HDMode, '/HD/fsm/mode', callback_group=reentrant_callback_group)

        # ===== ACTIONS =====

        self.hd_manipulation_action = ActionServer(self.node, HDManipulation, cs_names["HD"]["action"], self.model.HD.hd_manipulation_action,
                                        goal_callback=self.model.HD.hd_manipulation_goal_status)

        self.nav_reach_goal_action = ActionServer(self.node, NAVReachGoal, cs_names["NAV"]["action"], self.model.Nav.nav_reach_goal_action,
                                                  goal_callback=self.model.Nav.nav_reach_goal_status)

        self.drill_action_ = ActionServer(self.node, DrillCmd, cs_names["DRILL"]["action"], self.model.Drill.drill_action, 
                                          goal_callback=self.model.Drill.drill_goal_status)

        # ===== SUBSCRIBERS =====

        ### here we will define services and topics that the rover will subscribe to

        # -- CS messages --

        self.node.create_subscription(Joy, cs_names["NAV"]["topic_gamepad_nav"], self.transfer_gamepad_cmd_nav, 10)
        self.node.create_subscription(Joy, cs_names["HD"]["topic_gamepad_commands"], self.transfer_gamepad_cmd_hd, 10)

        self.node.create_subscription(String, '/ROVER/performance', self.model.update_metrics, 10)

        # -- SC messages --
        self.node.create_subscription(ScMotorStatus,         'SC/motor_status',            self.model.Drill.update_motor_status, 10)
        self.node.create_subscription(String,         'SC/fsm_status',            self.model.Drill.update_drill_status, 10)
        # self.node.create_subscription(Int8,               'SC/fsm_state_to_cs',      self.controller.science_state, 10)
        # self.node.create_subscription(Float32MultiArray,  'SC/motors_pos',           self.controller.science_motors_pos, 10)
        # self.node.create_subscription(Float32MultiArray,  'SC/motors_speed',         self.controller.science_motors_vels, 10)
        # self.node.create_subscription(Float32MultiArray,  'SC/motors_currents',      self.controller.science_motors_currents, 10)
        # self.node.create_subscription(Int8MultiArray,     'SC/limit_switches',       self.controller.science_limit_switches, 10)
        #self.node.create_subscription(MassArray,         'EL/container/mass',        self.controller.science_container_mass, 10)
        self.node.create_subscription(MassArray,         'EL/drill/mass',            self.model.Elec.update_mass_measurement, 10)
        # self.node.create_subscription(SpectroResponse,   'EL/spectro_response',   self.controller.science_spectrometer, 10)
        # self.node.create_subscription(NPK,               'EL/npk',            self.controller.science_npk, 10)
        # self.node.create_subscription(FourInOne,         'EL/four_in_one',    self.controller.science_4in1, 10)

        # -- HD messages --
        self.node.create_subscription(
            JointState,       '/HD/motor_control/joint_telemetry',   self.model.HD.hd_joint_state, 10)
        # self.node.create_subscription(
        #     Int8MultiArray,   'HD/ar_tags',           self.controller.hd_ARtags, 10)
        # self.node.create_subscription(
        #     Int8,             'HD/task_outcome',      self.controller.hd_task_outcome, 10)
        # self.node.create_subscription(
            # Voltage,          'EL/voltage',         self.controller.hd_voltage, 10)
        # self.node.create_subscription(String,        '/detected_panel',            self.controller.hd_set_ready, 10)

        # -- NAV messages --
        self.node.create_subscription(Odometry,         '/lio_sam/odom',                self.model.Nav.nav_odometry  , 10)
        # self.node.create_subscription(Path,             '/plan',                        self.controller.nav_path      , 10)
        self.node.create_subscription(Wheelstatus,      '/NAV/absolute_encoders',       self.model.Nav.nav_wheel, 10)
        self.node.create_subscription(Motorcmds,        '/NAV/displacement',            self.model.Nav.nav_displacement, 10)

        self.node.get_logger().info("Rover Node Started")

    # timer callback for sending rover state continuously
    def timer_callback(self):
        msg = String()
        self.rover_state_json["timestamp"] = int(time.time()) # epoch

        # Log in MongoDB
        rover_bson = json.loads(json_util.dumps(self.rover_state_json))
        self.logger.log(rover_bson)

        msg.data = json.dumps(self.rover_state_json)
        self.clear_rover_msgs() # clear temporary messages like errors and warnings
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
        

        # run ros
    def run(self):
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self.node)
        executor.spin()
        #thr = threading.Thread(target=executor.spin, daemon=True).start()
        #rclpy.spin(self.node)
        rclpy.shutdown()



def main():
    rover = RoverNode()
    rover.run()
