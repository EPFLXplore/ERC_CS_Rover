# ===============Node for interfacing between Rover and CS=================

import time
import rclpy
from rclpy.logging import LoggingSeverity
from rclpy.action import ActionServer
# import math
import sys

from std_msgs.msg       import Int8, Int16, Int32, Bool, String, Int8MultiArray,  Int16MultiArray, Float32MultiArray, UInt8MultiArray

# from geometry_msgs.msg  import Twist, PoseStamped
# from actionlib_msgs.msg import GoalID
from sensor_msgs.msg import JointState, Joy
from nav_msgs.msg import Odometry, Path
from rclpy.callback_groups import ReentrantCallbackGroup
# from diagnostic_msgs.msg import DiagnosticStatus

from custom_msg.msg import Wheelstatus, Motorcmds, MassArray
from custom_msg.action import HDManipulation, NAVReachGoal, DrillTerrain
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from custom_msg.srv import ChangeModeSystem

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
        self.node = rclpy.create_node('ROVER')

        self.model = NewModel(self)

        with open('/home/xplore/dev_ws/src/rover_pkg/rover_pkg/template_state.json') as json_file:
            self.rover_state_json = dict(json.load(json_file))
            
        #self.jetson = jtop()
        #self.jetson.attach(self.model.jetson_callback)
        #threading.Thread(target=self.jetson.loop_for_ever).start()

        # ==========================================================
        #              MESSAGES BETWEEN ROVER AND CS
        # ==========================================================

        # ===== PUBLISHERS =====

        self.rover_state_pub = self.node.create_publisher(String, 'Rover/RoverState', 1)
        self.timer = self.node.create_timer(0.1, self.timer_callback)

        # -- NAV messages --
        self.nav_cmd_pub = self.node.create_publisher(Joy, '/CS/NAV_gamepad', 1) # Name to be changed
        self.nav_mode_pub = self.node.create_publisher(Int8, '/ROVER/NAV_mode', 1)

        # -- HD messages --
        self.hd_cmd_pub = self.create_publisher(Float32MultiArray, "/CS/HD_gamepad", 10)
        # self.man_inv_axis_pub = self.create_publisher(Float32MultiArray, "/ROVER/HD_man_inv_axis", 10)
        self.hd_mode_pub = self.create_publisher(Int8, "/ROVER/HD_mode", 10)

        # ===== SERVICES =====

        self.change_rover_mode = self.node.create_service(ChangeModeSystem, "/Rover/ChangeModeSystem", self.model.change_mode_system_service, callback_group=ReentrantCallbackGroup())

        # ===== ACTIONS =====

        self.hd_manipulation_action = ActionServer(self.node, HDManipulation, "/Rover/HandlingDeviceManipulation", self.model.HD.hd_manipulation_action,
                                        goal_callback=self.model.HD.hd_manipulation_goal_status)

        self.nav_reach_goal_action = ActionServer(self.node, NAVReachGoal, "/Rover/NavigationReachGoal", self.model.Nav.nav_reach_goal_action,
                                                  goal_callback=self.model.Nav.nav_reach_goal_status)

        self.drill_action_ = ActionServer(self.node, DrillTerrain, "/Rover/DrillTerrain", self.model.Drill.drill_action, 
                                          goal_callback=self.model.Drill.drill_goal_status)

        # ===== SUBSCRIBERS =====

        ### here we will define services and topics that the rover will subscribe to

        # -- CS messages --

        self.node.create_subscription(Joy, 'CS/GamepadCmdsNavigation', self.transfer_gamepad_cmd_nav, 10)
        self.node.create_subscription(Joy, 'CS/GamepadCmdsHandlingDevice', self.transfer_gamepad_cmd_hd, 10)

        # SC --> Rover
        # self.node.create_subscription(Int8, 'SC/fsm_state_to_cs'          , self.model.SC.science_fsm_callback   , 10)  # self.SC_infos_pub.publish)

        # -- SC messages --
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




        ####### Also include a method for obtaining stats of the device (htop or jtop)

    # timer callback for sending rover state continuously
    def timer_callback(self):
        msg = String()
        msg.data = json.dumps(self.rover_state_json)
        self.rover_state_pub.publish(msg)

    def transfer_gamepad_cmd_nav(self, msg):
        # TODO: Check that the mode is set to MANUAL
        self.nav_cmd_pub.publish(msg)

    def transfer_gamepad_cmd_hd(self, msg):
        # TODO: Check that the mode is set to MANUAL
        axes = Float32MultiArray()
        axes.data = msg.axes

        # Reverse the axes for Joint 3 and Joint 4
        if msg.buttons[0] == 1:
            axes.data[2] = -axes.data[2]
        if msg.buttons[1] == 1:
            axes.data[3] = -axes.data[3]

        # add the gripper buttons to the axes
        gripper_vel = msg.buttons[2] - msg.buttons[3] if msg.buttons[2] or msg.buttons[3] else msg.buttons[4] - msg.buttons[5]
        axes.data.append(gripper_vel)

        self.hd_cmd_pub.publish(axes)
        

        # run ros
    def run(self):
        print("Rover node started !")
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self.node)
        executor.spin()
        #thr = threading.Thread(target=executor.spin, daemon=True).start()
        #rclpy.spin(self.node)
        rclpy.shutdown()



def main():
    rover = RoverNode()
    rover.run()