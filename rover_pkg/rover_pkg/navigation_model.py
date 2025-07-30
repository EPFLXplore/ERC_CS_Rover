from rclpy.action import GoalResponse
from nav_msgs.msg import Odometry
from custom_msg.action import NAVReachGoal
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from .states import SubSystems, Errors, LedMode

'''
Author: Giovanni Ranieri
Year: 2024-25
Description: Navigation Model. This class handles the navigation actions, feedback, and state management.
'''

class Navigation:
    def __init__(self, rover_node):
        self.rover_node = rover_node
        
        # Standard variables
        self.in_fault = False
        self.feedback = None
        self.running = False
        self.cancel_nav = False
        self.result = None
        self.counter_cancel = 0

        # info related to navigation parameters
        self.position = [0,0,0]
        self.orientation = [0,0,0]
        self.linVel = [0,0,0]
        self.angVel = [0,0,0]
        self.steering_wheel_ang = [0,0,0,0]
        self.driving_wheel_ang = [0,0,0,0]
        self.steering_wheel_state = [0,0,0,0]
        self.driving_wheel_state = [0,0,0,0]
        
        # Hardware values for the ERC 2025. Please consider to update them for new rovers.
        # They are used for the display at the CS.
        self.wheels_radius = 0.1325 # in [m]
        self.gear_ratio = 1.0/53.0

        # Subscription for the subsystem state
        self.rover_node.node.create_subscription(String, self.rover_node.nav_names['system_status'], self.handle_state, 10)
    
    def reset_informations(self):
        #self.rover_node.model.Elec.send_led_commands(SubSystems.NAVIGATION, 0)

        # front_left wheel
        self.rover_node.rover_state_json['navigation']['wheels']['front_left']['current_driving'] = "0.0"
        self.rover_node.rover_state_json['navigation']['wheels']['front_left']['current_steering'] = "0.0"
        self.rover_node.rover_state_json['navigation']['wheels']['front_left']['speed'] = "0.0"
        self.rover_node.rover_state_json['navigation']['wheels']['front_left']['steering_angle'] = "0.0"
        self.rover_node.rover_state_json['navigation']['wheels']['front_left']['steering_motor_state'] = False
        self.rover_node.rover_state_json['navigation']['wheels']['front_left']['driving_motor_state'] = False
        self.rover_node.rover_state_json['navigation']['wheels']['front_left']['steering_fault'] = False
        self.rover_node.rover_state_json['navigation']['wheels']['front_left']['driving_fault'] = False
        

        # front_right wheel
        self.rover_node.rover_state_json['navigation']['wheels']['front_right']['current_driving'] = "0.0"
        self.rover_node.rover_state_json['navigation']['wheels']['front_right']['current_steering'] = "0.0"
        self.rover_node.rover_state_json['navigation']['wheels']['front_right']['speed'] = "0.0"
        self.rover_node.rover_state_json['navigation']['wheels']['front_right']['steering_angle'] = "0.0"
        self.rover_node.rover_state_json['navigation']['wheels']['front_right']['steering_motor_state'] = False
        self.rover_node.rover_state_json['navigation']['wheels']['front_right']['driving_motor_state'] = False
        self.rover_node.rover_state_json['navigation']['wheels']['front_right']['steering_fault'] = False
        self.rover_node.rover_state_json['navigation']['wheels']['front_right']['driving_fault'] = False

        
        # back_right wheel
        self.rover_node.rover_state_json['navigation']['wheels']['rear_right']['current_driving'] = "0.0"
        self.rover_node.rover_state_json['navigation']['wheels']['rear_right']['current_steering'] = "0.0"
        self.rover_node.rover_state_json['navigation']['wheels']['rear_right']['speed'] = "0.0"
        self.rover_node.rover_state_json['navigation']['wheels']['rear_right']['steering_angle'] = "0.0"
        self.rover_node.rover_state_json['navigation']['wheels']['rear_right']['steering_motor_state'] = False
        self.rover_node.rover_state_json['navigation']['wheels']['rear_right']['driving_motor_state'] = False
        self.rover_node.rover_state_json['navigation']['wheels']['rear_right']['steering_fault'] = False
        self.rover_node.rover_state_json['navigation']['wheels']['rear_right']['driving_fault'] = False

        
        # back_left wheel
        self.rover_node.rover_state_json['navigation']['wheels']['rear_left']['current_driving'] = "0.0"
        self.rover_node.rover_state_json['navigation']['wheels']['rear_left']['current_steering'] = "0.0"
        self.rover_node.rover_state_json['navigation']['wheels']['rear_left']['speed'] = "0.0"
        self.rover_node.rover_state_json['navigation']['wheels']['rear_left']['steering_angle'] = "0.0"
        self.rover_node.rover_state_json['navigation']['wheels']['rear_left']['steering_motor_state'] = False
        self.rover_node.rover_state_json['navigation']['wheels']['rear_left']['driving_motor_state'] = False
        self.rover_node.rover_state_json['navigation']['wheels']['rear_left']['steering_fault'] = False
        self.rover_node.rover_state_json['navigation']['wheels']['rear_left']['driving_fault'] = False

        self.rover_node.rover_state_json['cameras']['navigation']['Front']['status'] = False
        self.rover_node.rover_state_json['cameras']['navigation']['Front']['node'] = False
        self.rover_node.rover_state_json['cameras']['navigation']['Front']['depth'] = False

    '''
    Function handling the state of the navigation subsystem. 
    If the state is 1, it means the navigation is activated, otherwise it is off.
    '''
    def handle_state(self, msg):
        self.rover_node.rover_state_json['rover']['status']['systems']['navigation']['status'] = msg.data
        
        if msg.data == 'Off' and self.rover_node.rover_state_json['rover']['status']['systems']['navigation']['status'] != 'Off':
            self.reset_informations()

    
    def nav_odometry(self, odometry):

        self.position = [odometry.pose.pose.position.x, odometry.pose.pose.position.y, odometry.pose.pose.position.z]
        #self.orientation = [odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w]
        #self.linVel = [odometry.twist.twist.linear.x, odometry.twist.twist.linear.y, odometry.twist.twist.linear.z]
        #self.angVel = [odometry.twist.twist.angular.x, odometry.twist.twist.angular.y, odometry.twist.twist.angular.z]

        # update the rover status
        self.rover_node.rover_state_json['navigation']['localization']['position']["x"] = round(self.position[0], 2)
        self.rover_node.rover_state_json['navigation']['localization']['position']["y"] = round(self.position[1], 2)

    '''
    Function callback for the navigation motors status.
    It updates the navigation state in the rover state JSON.
    '''
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
        
        if self.rover_node.rover_state_json['rover']['status']['systems']['navigation']['status'] == 'Off':
            return

        # conversion RPM tp m/s
        rps_to_ms = 2 * 3.1415 * self.wheels_radius / 60.0

        # states
        self.steering_wheel_state = msg.state[4:8]
        self.driving_wheel_state = msg.state[0:4]

        # currents
        self.steering_current = msg.current[4:8]
        self.driving_current = msg.current[0:4]
        
        # averaged current
        self.steering_average_current = msg.average_current[4:8]
        self.driving_average_current = msg.average_current[0:4]

        # position
        self.steering_wheel_ang = [float(i/65536 * 360) for i in msg.position[0:4]]

        # velocity
        self.driving_wheel_vel = [float(i * rps_to_ms * self.gear_ratio) for i in msg.velocity[0:4]]

        # fault
        self.fault_steering = msg.fault_state[4:8]
        self.fault_driving = msg.fault_state[0:4]

        # update the rover status

        # front_left wheel
        self.rover_node.rover_state_json['navigation']['wheels']['front_left']['current_driving'] = abs(self.driving_average_current[0])
        self.rover_node.rover_state_json['navigation']['wheels']['front_left']['current_steering'] = abs(self.steering_average_current[0])
        self.rover_node.rover_state_json['navigation']['wheels']['front_left']['speed'] = abs(round(self.driving_wheel_vel[0], 1))
        self.rover_node.rover_state_json['navigation']['wheels']['front_left']['steering_angle'] = int(self.steering_wheel_ang[0])
        self.rover_node.rover_state_json['navigation']['wheels']['front_left']['steering_motor_state'] = self.steering_wheel_state[0]
        self.rover_node.rover_state_json['navigation']['wheels']['front_left']['driving_motor_state'] = self.driving_wheel_state[0]
        self.rover_node.rover_state_json['navigation']['wheels']['front_left']['steering_fault'] = self.fault_steering[0]
        self.rover_node.rover_state_json['navigation']['wheels']['front_left']['driving_fault'] = self.fault_driving[0]


        # front_right wheel
        self.rover_node.rover_state_json['navigation']['wheels']['front_right']['current_driving'] = abs(self.driving_average_current[1])
        self.rover_node.rover_state_json['navigation']['wheels']['front_right']['current_steering'] = abs(self.steering_average_current[1])
        self.rover_node.rover_state_json['navigation']['wheels']['front_right']['speed'] = abs(round(self.driving_wheel_vel[1], 1))
        self.rover_node.rover_state_json['navigation']['wheels']['front_right']['steering_angle'] = int(self.steering_wheel_ang[1])
        self.rover_node.rover_state_json['navigation']['wheels']['front_right']['steering_motor_state'] = self.steering_wheel_state[1]
        self.rover_node.rover_state_json['navigation']['wheels']['front_right']['driving_motor_state'] = self.driving_wheel_state[1]
        self.rover_node.rover_state_json['navigation']['wheels']['front_right']['steering_fault'] = self.fault_steering[1]
        self.rover_node.rover_state_json['navigation']['wheels']['front_right']['driving_fault'] = self.fault_driving[1]

        
        # back_right wheel
        self.rover_node.rover_state_json['navigation']['wheels']['rear_right']['current_driving'] = abs(self.driving_average_current[2])
        self.rover_node.rover_state_json['navigation']['wheels']['rear_right']['current_steering'] = abs(self.steering_average_current[2])
        self.rover_node.rover_state_json['navigation']['wheels']['rear_right']['speed'] = abs(round(self.driving_wheel_vel[2], 1))
        self.rover_node.rover_state_json['navigation']['wheels']['rear_right']['steering_angle'] = int(self.steering_wheel_ang[2])
        self.rover_node.rover_state_json['navigation']['wheels']['rear_right']['steering_motor_state'] = self.steering_wheel_state[2]
        self.rover_node.rover_state_json['navigation']['wheels']['rear_right']['driving_motor_state'] = self.driving_wheel_state[2]
        self.rover_node.rover_state_json['navigation']['wheels']['rear_right']['steering_fault'] = self.fault_steering[2]
        self.rover_node.rover_state_json['navigation']['wheels']['rear_right']['driving_fault'] = self.fault_driving[2]

        
        # back_left wheel
        self.rover_node.rover_state_json['navigation']['wheels']['rear_left']['current_driving'] = abs(self.driving_average_current[3])
        self.rover_node.rover_state_json['navigation']['wheels']['rear_left']['current_steering'] = abs(self.steering_average_current[3])
        self.rover_node.rover_state_json['navigation']['wheels']['rear_left']['speed'] = abs(round(self.driving_wheel_vel[3], 1))
        self.rover_node.rover_state_json['navigation']['wheels']['rear_left']['steering_angle'] = int(self.steering_wheel_ang[3])
        self.rover_node.rover_state_json['navigation']['wheels']['rear_left']['steering_motor_state'] = self.steering_wheel_state[3]
        self.rover_node.rover_state_json['navigation']['wheels']['rear_left']['driving_motor_state'] = self.driving_wheel_state[3]
        self.rover_node.rover_state_json['navigation']['wheels']['rear_left']['steering_fault'] = self.fault_steering[3]
        self.rover_node.rover_state_json['navigation']['wheels']['rear_left']['driving_fault'] = self.fault_driving[3]

        # Check if motor is in fault:
        if any(self.fault_steering) or any(self.fault_driving):
            
            # If the state is not in fault we update
            if not self.in_fault:
                #self.rover_node.model.Elec.send_led_errors(SubSystems.NAVIGATION, Errors.FAULT.value)
                self.in_fault = True
        else:
            
            # If the state was in fault we update
            if self.in_fault:
                if self.rover_node.rover_state_json['rover']['status']['systems']['navigation']['status'] == 'Ackermann':
                    pass
                    #self.rover_node.model.Elec.send_led_commands(SubSystems.NAVIGATION, 1)
                elif self.rover_node.rover_state_json['rover']['status']['systems']['navigation']['status'] == 'Omni':
                    pass
                    #self.rover_node.model.Elec.send_led_commands(SubSystems.NAVIGATION, 2)
                elif self.rover_node.rover_state_json['rover']['status']['systems']['navigation']['status'] == 'Auto':
                    pass
                    #self.rover_node.model.Elec.send_led_commands(SubSystems.NAVIGATION, 3)
                
                self.in_fault = False

    def feedback_odometry(self, pose_stamped):
        msg = Odometry()
        msg.header.stamp = self.rover_node.node.get_clock().now().to_msg()
        msg.pose.pose.position.x = pose_stamped.pose.position.x
        msg.pose.pose.position.y = pose_stamped.pose.position.y
        msg.pose.pose.position.z = pose_stamped.pose.position.z
        msg.pose.pose.orientation.x = pose_stamped.orientation.x
        msg.pose.pose.orientation.y = pose_stamped.orientation.y
        msg.pose.pose.orientation.z = pose_stamped.orientation.z
        msg.pose.pose.orientation.w = pose_stamped.orientation.w
        return msg
    
    def action_status(self, goal):
        if self.rover_node.rover_state_json['rover']['status']['systems']['navigation']['status'] == 'Off':
            self.get_logger().info('Received goal from NAV but rejeected...')
            return GoalResponse.REJECT
        
        self.get_logger().info('Received goal from NAV...')
        return GoalResponse.ACCEPT

    def make_action(self, goal_handle):
        self.goal_handle_cs = goal_handle
        self.rover_node.node.get_logger().info("NAV Reach Goal action starting")

        nav_goal = self.create_nav_goal(self.create_pose_stamped(
            self.goal_handle_cs.request.goal.x,
            self.goal_handle_cs.request.goal.y,
            self.goal_handle_cs.request.goal.theta
        ))
       
        self.rover_node.nav_action_client.wait_for_server()
        future_c = self.rover_node.nav_action_client.send_goal_async(nav_goal, 
                                self.feedback_nav_to_cs)
        
        future_c.add_done_callback(self.nav_response_callback)

        while self.running:
            continue

        if not self.cancel_nav:
            self.rover_node.node.get_logger().info("Nav Goal finished successfully")
            return self.result_nav_action("Drill Goal finished successfully", self.default_pos_return(), 0, "no errors")
        else:
            self.rover_node.node.get_logger().info("Canceled goal NAV successfull")
            return self.result_nav_action("Drill Goal finished successfully", self.default_pos_return(), 0, "no errors")
    
    def nav_response_callback(self, future):
        self.goal_handle_nav = future.result()

        # GOAL REJECTED FROM NAV - FORWARD TO CS (return is sufficient? need to test)

        if not self.goal_handle_nav.accepted:
            self.cancel_nav = True
            self.running = False
            self.rover_node.node.get_logger().info('Nav Goal rejected from nav2')
            return self.result_nav_action("Nav Goal rejected from Nav2", self.default_pos_return(), 1, '')

        self.rover_node.node.get_logger().info('Goal accepted for nav2')
        
        get_result_future = self.goal_handle_nav.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    '''
    Function handling the result of the action to the Drill. Return the result and the status to the CS as an object
    '''
    def result_callback(self, future):
        self.result = future.result().result
        self.feedback = None

        if not self.cancel_nav:
            self.goal_handle_cs.succeed()

        self.running = False

    def feedback_nav_to_cs(self, navigate_to_goal_feedback):

        if self.cancel_nav and self.counter_cancel == 0:
            self.counter_cancel = self.counter_cancel + 1
            future_drill = self.goal_handle_nav.cancel_goal_async()
            future_drill.add_done_callback(self.cancel_nav_action)
        
        else:
            self.feedback = self.create_feedback_to_cs("", navigate_to_goal_feedback.feedback, 0, "no warning")
            self.goal_handle_cs.publish_feedback(self.feedback)


    '''
    Cancel action from CS. Need to send cancellation to NAV and forward cancellation
    '''
    def cancel_goal(self, goal_handle_cs):
        self.rover_node.node.get_logger().info("Nav goal cancelation requested...")
        self.cancel_nav = True

    '''
    Cancel action from ROVER. DONT KNOW IF IT WILL WORK BECAUSE OF CALLBACK RETURN
    '''
    def cancel_nav_action(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.rover_node.node.get_logger().info('Nav Goal successfully canceled')
        else:
            self.rover_node.node.get_logger().error('Nav Goal failed to cancel...')
    
    def create_nav_goal(self, pose_stamped):
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = pose_stamped
        nav_goal.behaviour_tree = ''

        return pose_stamped

    def create_pose_stamped(self, point_x, point_y, orientation):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.pose.position.x = point_x
        pose_stamped.pose.position.y = point_y
        pose_stamped.pose.orientation.w = orientation

        return pose_stamped
    
    def result_nav_action(self, resultt, final_pos, error_type, error_messsage):
        result = NAVReachGoal.Result()
        result.result = resultt
        result.final_pos = final_pos
        result.error_type = error_type
        result.error_message = error_messsage
        return result
    
    def create_feedback_to_cs(self, status, cs_feedback, error_type, error_message):
        feedback = NAVReachGoal.Feedback()
        feedback.current_status = status
        feedback.current_pos = self.feedback_odometry(cs_feedback.current_pos)
        feedback.distance_to_goal = cs_feedback.distance_remaining
        feedback.warning_type = error_type
        feedback.warning_message = error_message

    def default_pos_return(self):
        msg = Odometry()
        msg.header.stamp = self.rover_node.node.get_clock().now().to_msg()
        msg.pose.pose.position.x = 0
        msg.pose.pose.position.y = 0
        msg.pose.pose.position.z = 0
        msg.pose.pose.orientation.x = 0
        msg.pose.pose.orientation.y = 0
        msg.pose.pose.orientation.z = 0
        msg.pose.pose.orientation.w = 0
        return msg
    
    '''
    Forward the speed of the rover to NAV
    '''
    def change_speed_rover(self, msg):
        if(msg.data <= 0.5 or msg.data >= 2.31): return
        
        self.rover_node.speed_rover_pub.publish(msg)