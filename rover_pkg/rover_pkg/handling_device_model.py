from rclpy.action import GoalResponse
from custom_msg.action import HDManipulation, NewHDGoal
from custom_msg.srv import RequestHDGoal
import math
from custom_msg.msg import HDGoal
from std_msgs.msg import String

class HandlingDevice:
    def __init__(self, rover_node):
        self.rover_node = rover_node

        self.running = False
        self.feedback = None
        self.cancel_hd = False
        self.result = None
        self.counter_cancel = 0

        self.rover_node.node.create_subscription(String, self.rover_node.hd_names['system_status'], self.handle_state, 10)

    def reset_informations(self):
        self.rover_node.model.Elec.send_led_commands("hd", "Off")

        self.rover_node.rover_state_json['handling_device']['state']['current_command'] = "NONE"
        self.rover_node.rover_state_json['handling_device']['state']['task'] = "NONE" 
        for i in range(7):
                self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['angle'] = '0.0'
                self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['velocity'] = '0.0'
                self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['current'] = '0.0'
                self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['torque'] = '0.0'
                self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['state'] = False
                self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['mode_motor'] = 0
    
    def make_action(self, goal_handle_cs):
        self.goal_handle_cs = goal_handle_cs
        self.rover_node.node.get_logger().info("HD action starting... ")

        # Create action for HD

        goal = self.createHdGoal(goal_handle_cs.request.action)

        self.rover_node.hd_action_client.wait_for_server()
        future_c = self.rover_node.hd_action_client.send_goal_async(goal, 
                                self.feedback_callback)
        
        future_c.add_done_callback(self.hd_response_callback)
        self.running = True
        
        while self.running:
            continue

        if not self.cancel_hd:
            self.rover_node.node.get_logger().info('FINISHED')
        
        return self.result_hd_action(self.result)

    
    def feedback_callback(self, feedback):
        if self.cancel_hd and self.counter_cancel == 0:
            self.counter_cancel = self.counter_cancel + 1
            future_hd = self.goal_handle_hd.cancel_goal_async()
            future_hd.add_done_callback(self.cancel_hd_action)
        
        else:
            self.feedback = feedback.feedback
            self.update_hd_feedback(self.feedback)
    

    def action_status(self, goal):
        if self.rover_node.rover_state_json['rover']['status']['systems']['handling_device']['status'] == 'Off':
            return GoalResponse.REJECT
        
        self.result = None
        self.running = True
        self.feedback = None
        self.cancel_hd = False
        self.counter_cancel = 0
        return GoalResponse.ACCEPT
    
    def update_hd_feedback(self, feedback):
        self.rover_node.rover_state_json['handling_device']['state']['current_command'] = feedback.current_command
        self.rover_node.rover_state_json['handling_device']['state']['task'] = feedback.task


    def result_hd_action(self, result_action):
        result = HDManipulation.Result()
        result.result = result_action.result
        result.error_type = result_action.error_type
        result.error_message = result_action.error_message
        self.rover_node.rover_state_json['handling_device']['state']['current_command'] = "NONE"
        self.rover_node.rover_state_json['handling_device']['state']['task'] = "NONE" 
        return result
    
    '''
    Cancel action from ROVER.
    '''
    def cancel_hd_action(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.rover_node.node.get_logger().info('HD Goal successfully canceled')
        else:
            self.rover_node.node.get_logger().error('HD Goal failed to cancel...')
            # if enter here.. bad for us
    
    '''
    Function handling the response of the request to the Drill.
    '''
    def hd_response_callback(self, future):
        self.goal_handle_hd = future.result()

        # GOAL REJECTED FROM HD - FORWARD TO CS (return is sufficient? need to test)

        if not self.goal_handle_hd.accepted:
            self.cancel_hd = True
            self.running = False
            self.rover_node.node.get_logger().info('HD Goal rejected from HD')
            return self.result_hd_action(self.result)

        self.rover_node.node.get_logger().info('HD Goal accepted from HD')
        self.rover_node.model.Elec.send_led_commands("hd", "action")
        
        get_result_future = self.goal_handle_hd.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    
    def createHdGoal(self, action):
        goal = NewHDGoal.Goal()
        msg_goal = HDGoal()
        
        # Predefined poses
        if action == HDGoal.HOME or action == HDGoal.ZERO or action == HDGoal.COBRA or action == HDGoal.ABOVE_GROUND:
            msg_goal.target = HDGoal.NAMED_POSE
            msg_goal.predefined_pose = action

        # Tool Actions
        elif action == HDGoal.SHOVEL_TOOL:
            msg_goal.target = HDGoal.TOOL_PICKUP
            msg_goal.target = action
            
        else:
            msg_goal.target = action

        goal.goal = msg_goal
        return goal
    
    def result_callback(self, future):
        self.result = future.result().result
        self.feedback = None

        if not self.cancel_hd:
            self.goal_handle_cs.succeed()

        self.running = False
        self.rover_node.model.Elec.send_led_commands("hd", "On")   
    
    '''
    Cancel action from CS. Need to send cancellation to HD and forward cancellation
    '''
    def cancel_goal_from_cs(self, goal_handle_cs):
        self.rover_node.node.get_logger().info("HD goal cancelation requested...")
        self.cancel_hd = True

    # -----------------------------------------------------------------------------

    def handle_state(self, msg):
        self.rover_node.rover_state_json['rover']['status']['systems']['handling_device']['status'] = msg.data

        if msg.data == 'Off':
           self.reset_informations()

    def hd_motor_cmds(self, msg):

        currents = msg.current
        motor_mode = msg.motor_mode
        positions = msg.position
        vel = msg.velocity
        #torque = msg.torque

        # This mapping comes from the ETH Library of the Arm
        # There are the states of the motors
        mapping = {0: "NotReadyToSwitchOn",
                   1: "SwitchOnDisabled",
                   2: "ReadyToSwitchOn",
                   3: "SwitchedOn",
                   4: "OperationEnabled",
                   5: "QuickStopActive", 
                   6: "FaultReactionActive",
                   7: "Fault",
                   8: "NA"}

        # update the rover status
        for i in range(7):
            self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['angle'] = round(math.degrees(positions[i]), 1)
            self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['velocity'] = abs(vel[i])
            self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['current'] = abs(round(currents[i], 1))
            #self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['torque'] = abs(torque[i])
            self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['mode_motor'] = mapping[motor_mode[i]]