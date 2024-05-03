from custom_msg.msg import Wheelstatus, Motorcmds
import numpy as np

class NewModel:
    def __init__(self, rover_node):
        self.rover_node = rover_node

        # self.drill = Drill(rover_node)
        self.HD = HandlingDevice(rover_node)
        self.Nav = Navigation(rover_node)
        # self.Cams = Cameras(rover_node)
        self.Elec = Elec(rover_node)

    def jetson_callback(self):
        self.rover_state_json['rover']['hardware']['json'] = self.rover_node.jetson.json()


class HandlingDevice:
    def __init__(self, rover_node):
        self.rover_node = rover_node

        # info related to handling device parameters
        # self.joint_telemetry = [0,0,0,0,0,0]
        # self.joint_command = [0,0,0,0,0,0]
        # self.joint_state = [0,0,0,0,0,0]

        # HD --> Rover
        #self.node.create_subscription(JointState, 'HD/motor_control/joint_telemetry', self.update_hd_joint_telemetry , 10)

    def hd_joint_state(self, joint_state):

        self.joint_positions = joint_state.position
        self.joint_velocities = joint_state.velocity
        self.joint_current = joint_state.effort

        # update the rover status
        for i in range(len(self.joint_positions)):
            self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['angle'] = self.joint_positions[i]
            self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['velocity'] = self.joint_velocities[i]
            self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['current'] = self.joint_current[i]

# class Drill:

class Navigation:
    def __init__(self, rover_node):
        self.rover_node = rover_node

        # info related to navigation parameters
        self.position = [0,0,0]
        self.orientation = [0,0,0]
        self.linVel = [0,0,0]
        self.angVel = [0,0,0]
        self.steering_wheel_ang = [0,0,0,0]
        self.driving_wheel_ang = [0,0,0,0]
        self.steering_wheel_state = [0,0,0,0]
        self.driving_wheel_state = [0,0,0,0]

        # NAV --> Rover
        #self.node.create_subscription(PoseStamped,        '/lio_sam/current_pose'          , self.NAV_odometry_pub.publish , 10) # CS DIRECTLY SUBSCRIBED


    # def update_hd_joint_telemetry(self, msg):
    def nav_odometry(self, odometry):

        self.position = [odometry.pose.pose.position.x, odometry.pose.pose.position.y, odometry.pose.pose.position.z]

        self.orientation = [odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w]

        self.linVel = [odometry.twist.twist.linear.x, odometry.twist.twist.linear.y, odometry.twist.twist.linear.z]
        self.angVel = [odometry.twist.twist.angular.x, odometry.twist.twist.angular.y, odometry.twist.twist.angular.z]

        # update the rover status
        self.rover_node.rover_state_json['navigation']['localization']['position']["x"] = self.position[0]
        self.rover_node.rover_state_json['navigation']['localization']['position']["y"] = self.position[1]
        self.rover_node.rover_state_json['navigation']['localization']['position']["z"] = self.position[2]

        self.rover_node.rover_state_json['navigation']['localization']['orientation']["x"] = self.orientation[0]
        self.rover_node.rover_state_json['navigation']['localization']['orientation']["y"] = self.orientation[1]
        self.rover_node.rover_state_json['navigation']['localization']['orientation']["z"] = self.orientation[2]
        self.rover_node.rover_state_json['navigation']['localization']['orientation']["w"] = self.orientation[3]

        self.rover_node.rover_state_json['navigation']['localization']['linear_velocity']["x"] = self.linVel[0]
        self.rover_node.rover_state_json['navigation']['localization']['linear_velocity']["y"] = self.linVel[1]
        self.rover_node.rover_state_json['navigation']['localization']['linear_velocity']["z"] = self.linVel[2]

        self.rover_node.rover_state_json['navigation']['localization']['angular_velocity']["x"] = self.angVel[0]
        self.rover_node.rover_state_json['navigation']['localization']['angular_velocity']["y"] = self.angVel[1]
        self.rover_node.rover_state_json['navigation']['localization']['angular_velocity']["z"] = self.angVel[2]


    def nav_wheel(self, msg):
        """
        FRONT_LEFT_DRIVE = 0
        FRONT_RIGHT_DRIVE = 1
        BACK_RIGHT_DRIVE = 2
        BACK_LEFT_DRIVE = 3
        FRONT_LEFT_STEER = 4
        FRONT_RIGHT_STEER = 5
        BACK_RIGHT_STEER = 6
        BACK_LEFT_STEER = 7
        """
        # print(msg.state)
        #self.navigation.wheels_ang = []
        self.steering_wheel_ang = [float(i/65536 * 360) for i in msg.data[0:4]]
        self.driving_wheel_ang = [float(i/65536 * 360) for i in msg.data[4:8]]
        self.steering_wheel_state = msg.state[0:4]
        self.driving_wheel_state = msg.state[4:8]

        # self.... = msg.current # 8 elements in the array

        # update the rover status

        # front_left wheel
        self.rover_node.rover_state_json['navigation']['wheels']['front_left']['steering_angle'] = self.steering_wheel_ang[0]
        self.rover_node.rover_state_json['navigation']['wheels']['front_left']['steering_motor_state'] = self.steering_wheel_state[0]
        self.rover_node.rover_state_json['navigation']['wheels']['front_left']['driving_wheel_state'] = self.driving_wheel_state[0]
        # front_right wheel
        self.rover_node.rover_state_json['navigation']['wheels']['front_right']['steering_angle'] = self.steering_wheel_ang[1]
        self.rover_node.rover_state_json['navigation']['wheels']['front_right']['steering_motor_state'] = self.steering_wheel_state[1]
        self.rover_node.rover_state_json['navigation']['wheels']['front_right']['driving_wheel_state'] = self.driving_wheel_state[1]

        # back_right wheel
        self.rover_node.rover_state_json['navigation']['wheels']['rear_right']['steering_angle'] = self.steering_wheel_ang[2]
        self.rover_node.rover_state_json['navigation']['wheels']['rear_right']['steering_motor_state'] = self.steering_wheel_state[2]
        self.rover_node.rover_state_json['navigation']['wheels']['rear_right']['driving_wheel_state'] = self.driving_wheel_state[2]

        # back_left wheel
        self.rover_node.rover_state_json['navigation']['wheels']['rear_left']['steering_angle'] = self.steering_wheel_ang[3]
        self.rover_node.rover_state_json['navigation']['wheels']['rear_left']['steering_motor_state'] = self.steering_wheel_state[3]
        self.rover_node.rover_state_json['navigation']['wheels']['rear_left']['driving_wheel_state'] = self.driving_wheel_state[3]
    

    def nav_displacement(self, displacement):
        self.displacement_mode = displacement.modedeplacement
        self.info = displacement.info

class Elec:
    def __init__(self, rover_node):
        self.rover_node = rover_node

        # EL --> Rover
        # self.node.create_subscription(Voltage, 'EL/voltage', self.update_battery_voltage , 10)

    def update_mass_measurement(self, msg):
        self.rover_node.rover_state_json['electronics']['sensors']['mass_sensor']["drill"] = msg.mass[1]
        # self.rover_node.rover_state_json['electronics']['sensors']['mass_sensor']["drill"] = msg.mass[1]
        # self.rover_node.rover_state_json['electronics']['sensors']['mass_sensor']["drill"] = msg.mass[1]