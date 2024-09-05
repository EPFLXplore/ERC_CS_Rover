from rclpy.action import GoalResponse, CancelResponse
from nav_msgs.msg import Odometry
from custom_msg.action import NAVReachGoal
# from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import time

class Navigation:
    def __init__(self, rover_node):
        self.rover_node = rover_node

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