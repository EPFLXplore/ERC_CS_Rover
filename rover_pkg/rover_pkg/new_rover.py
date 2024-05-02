# ===============Node for interfacing between Rover and CS=================

import time
import rclpy
from rclpy.logging import LoggingSeverity
# import math
import sys

from std_msgs.msg       import Int8, Int16, Int32, Bool, String, Int8MultiArray,  Int16MultiArray, Float32MultiArray, UInt8MultiArray

# from geometry_msgs.msg  import Twist, PoseStamped
# from actionlib_msgs.msg import GoalID
from sensor_msgs.msg import JointState, Joy
from nav_msgs.msg import Odometry, Path
# from diagnostic_msgs.msg import DiagnosticStatus

from custom_msg.msg import Wheelstatus, Motorcmds

# from std_srvs.srv import SetBool
import json

from jtop import jtop
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
        
        self.jetson = jtop()
        self.jetson.attach(self.model.jetson_callback)
        threading.Thread(target=self.jetson.loop_for_ever).start()

        with open('/home/xplore/dev_ws/src/rover_pkg/rover_pkg/template_state.json') as json_file:
            self.rover_state_json = dict(json.load(json_file))

        # ==========================================================
        #              MESSAGES BETWEEN ROVER AND CS
        # ==========================================================

        # ===== PUBLISHERS =====

        # publish rover state continuously via JSON
        self.rover_state_pub = self.node.create_publisher(String, 'Rover/RoverState', 1)
        timer_period = 0.1  # seconds
        self.timer = self.node.create_timer(timer_period, self.timer_callback)

        # ===== SUBSCRIBERS =====

        ### here we will define services and topics that the rover will subscribe to

        # SC --> Rover
        # self.node.create_subscription(Int8, 'SC/fsm_state_to_cs'          , self.model.SC.science_fsm_callback   , 10)  # self.SC_infos_pub.publish)

        # NAV --> Rover
        #self.node.create_subscription(PoseStamped,        '/lio_sam/current_pose'          , self.NAV_odometry_pub.publish , 10) # CS DIRECTLY SUBSCRIBED

        # # HD --> Rover
        # self.node.create_subscription(JointState, 'HD/motor_control/joint_telemetry', self.update_hd_joint_telemetry , 10)
    
        # dummy publisher
        # self.dummy_pub = self.node.create_publisher(JointState, 'HD/motor_control/joint_telemetry', 10)

        # -- SC messages --
        # self.node.create_subscription(Int8,               'SC/fsm_state_to_cs',      self.controller.science_state, 10)
        # self.node.create_subscription(Float32MultiArray,  'SC/motors_pos',           self.controller.science_motors_pos, 10)
        # self.node.create_subscription(Float32MultiArray,  'SC/motors_speed',         self.controller.science_motors_vels, 10)
        # self.node.create_subscription(Float32MultiArray,  'SC/motors_currents',      self.controller.science_motors_currents, 10)
        # self.node.create_subscription(Int8MultiArray,     'SC/limit_switches',       self.controller.science_limit_switches, 10)
        # self.node.create_subscription(MassArray,         'EL/container/mass',        self.controller.science_container_mass, 10)
        # self.node.create_subscription(MassArray,         'EL/drill/mass',            self.controller.science_drill_mass, 10)
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
        # print(self.rover_state_json['handling_device'])
        msg = String()
        msg.data = json.dumps(self.rover_state_json)
        self.rover_state_pub.publish(msg)

        # # dummy update for testing
        # t2 = time.time() - self.t1

        # d = JointState()
        # vals = [math.sin(t2), math.cos(t2), math.tan(t2)]
        # d.position = vals
        # self.dummy_pub.publish(d)

    



    # def update_hd_joint_telemetry(self, msg):
    #     positions = msg.position
    #     velocities = msg.velocity

    #     for i in range(len(positions)):
    #         self.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['angle'] = positions[i]
        

        # run ros
    def run(self):
        print("Rover node started !")
        rclpy.spin(self.node)
        rclpy.shutdown()



def main():
    rover = RoverNode()
    rover.run()




