from custom_msg.action import HDManipulation, NewHDGoal
from .states import SubSystems, Errors, LedMode
import math
from custom_msg.msg import HDGoal
from std_msgs.msg import String
from rclpy.action import GoalResponse, CancelResponse

class HandlingDevice:
    def __init__(self, rover_node):
        self.rover_node = rover_node
        
        self.in_fault = False

        self.running = False
        self.feedback = None
        self.cancel_hd = False
        self.result = None
        self.counter_cancel = 0

        # Switches
        self.switches = [value for key, value in vars(HDGoal).items() if key.startswith("BUTTON")]
        
        # Predefined poses
        self.predefined_poses = [HDGoal.FRONT_PANEL, HDGoal.RANGEMENT, HDGoal.HOME, HDGoal.ZERO, 
                                 HDGoal.COBRA, HDGoal.ABOVE_GROUND, HDGoal.PROBE_1, HDGoal.PROBE_2,
                                 HDGoal.PROBE_3]
        
        # Tools
        self.tools = [HDGoal.CLAM_TOOL]

        self.rover_node.node.create_subscription(String, self.rover_node.hd_names['system_status'], self.handle_state, 10)

    def reset_informations(self):
        #self.rover_node.model.Elec.send_led_commands(SubSystems.HANDLING_DEVICE, 0)
        self.rover_node.rover_state_json['handling_device']['state']['current_command'] = "NONE"
        self.rover_node.rover_state_json['handling_device']['state']['task'] = "NONE" 
        for i in range(7):
                self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['angle'] = '0.0'
                self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['velocity'] = '0.0'
                self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['current'] = '0.0'
                self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['torque'] = '0.0'
                self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['state'] = False
                self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['mode_motor'] = "NotReadyToSwitchOn"
    
    def make_action(self, goal_handle_cs):
        self.goal_handle_cs = goal_handle_cs
        self.rover_node.node.get_logger().info("HD action starting... ")

        # Create action for HD
        goal = self.createHdGoal(goal_handle_cs.request.action)

        # Start the second action and add the callback
        self.rover_node.hd_action_client.wait_for_server()
        future_c = self.rover_node.hd_action_client.send_goal_async(goal, 
                                self.feedback_callback)
        
        future_c.add_done_callback(self.hd_response_callback)
        self.running = True
        
        # Wait for the action to finish
        while self.running:
            continue

        # If the action was not canceled, we need to set the result
        if not self.cancel_hd:
            self.rover_node.node.get_logger().info('FINISHED')
            return self.result_hd_action(self.result)
        else:
            self.rover_node.node.get_logger().info('CANCELED')
            return self.no_result()
    
    def feedback_callback(self, feedback):
        
        # If the action is canceled, we need to cancel the goal HD
        if self.cancel_hd and self.counter_cancel == 0:
            self.counter_cancel = self.counter_cancel + 1
            
            # cancel the HD goal and wait response in the callback
            future_hd = self.goal_handle_hd.cancel_goal_async()
            future_hd.add_done_callback(self.cancel_hd_action)
        
        # else we just update the feedback
        else:
            self.feedback = feedback.feedback
            self.update_hd_feedback(self.feedback)
    

    # Accept or reject the goal of CS
    def action_status(self, goal):
        if self.rover_node.rover_state_json['rover']['status']['systems']['handling_device']['status'] == 'Off':
            return GoalResponse.REJECT
        
        self.result = None
        self.running = True
        self.feedback = None
        self.cancel_hd = False
        self.counter_cancel = 0
        return GoalResponse.ACCEPT
    
    # Update the feedback on the rover state
    def update_hd_feedback(self, feedback):
        self.rover_node.rover_state_json['handling_device']['state']['current_command'] = feedback.current_command
        self.rover_node.rover_state_json['handling_device']['state']['task'] = feedback.task

    # Create the result of the action
    def result_hd_action(self, result_action):
        result = HDManipulation.Result()
        result.result = result_action.result
        result.error_type = result_action.error_type
        result.error_message = result_action.error_message
        self.rover_node.rover_state_json['handling_device']['state']['current_command'] = "NONE"
        self.rover_node.rover_state_json['handling_device']['state']['task'] = "NONE" 
        return result
    
     # Create an empty result
    def no_result(self):
        result = HDManipulation.Result()
        result.result = "result_action.result"
        result.error_type = 1
        result.error_message = ""
        self.rover_node.rover_state_json['handling_device']['state']['current_command'] = "NONE"
        self.rover_node.rover_state_json['handling_device']['state']['task'] = "NONE" 
        return result
    
    # Callback for the cancelation of the HD action
    def cancel_hd_action(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.rover_node.node.get_logger().info('HD Goal successfully canceled')
        
            self.goal_handle_cs.canceled()
        
            # After receiving ack from HD, we exit the action running
            self.running = False
        else:
            self.rover_node.node.get_logger().error('HD Goal failed to cancel...')
            # if enter here.. bad for us lol
    
    '''
    Function handling the response of the request to the Drill.
    '''
    def hd_response_callback(self, future):
        self.goal_handle_hd = future.result()

        if not self.goal_handle_hd.accepted:
            self.cancel_hd = True
            self.running = False
            self.rover_node.node.get_logger().info('HD Goal rejected from HD')
            return self.result_hd_action(self.result)

        self.rover_node.node.get_logger().info('HD Goal accepted from HD')
        
        get_result_future = self.goal_handle_hd.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    
    def createHdGoal(self, action):
        goal = NewHDGoal.Goal()
        msg_goal = HDGoal()
        
        # Predefined poses
        if action in self.predefined_poses:
            msg_goal.target = HDGoal.NAMED_POSE
            msg_goal.predefined_pose = action

        # Tools
        elif action in self.tools:
            msg_goal.target = HDGoal.TOOL_PICKUP
            msg_goal.tool = action
            
        # Switches
        elif action in self.switches:
            msg_goal.target = HDGoal.BUTTON_TASK
            msg_goal.switch_name = action
            
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
    
    '''
    Cancel action from CS. Need to send cancellation to HD and forward cancellation
    '''
    def cancel_goal_from_cs(self, goal_handle_cs):
        self.rover_node.node.get_logger().info("HD goal cancelation requested...")
        self.cancel_hd = True
        
        return CancelResponse.ACCEPT

    # -----------------------------------------------------------------------------

    def handle_state(self, msg):
        self.rover_node.rover_state_json['rover']['status']['systems']['handling_device']['status'] = msg.data

        if msg.data == 'Off':
           self.reset_informations()

    def hd_motor_cmds(self, msg):
        
        if self.rover_node.rover_state_json['rover']['status']['systems']['handling_device']['status'] == 'Off':
            return

        currents = msg.current
        motor_mode = msg.motor_mode
        positions = msg.position
        vel = msg.velocity
        torque = msg.torque

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

        if(len(positions) == 7):
            # update the rover status
            for i in range(len(positions)):
                self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['angle'] = round(math.degrees(positions[i]), 1)
                self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['velocity'] = abs(vel[i])
                self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['current'] = abs(round(currents[i], 1))
                self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['torque'] = abs(torque[i])
                self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['mode_motor'] = mapping[motor_mode[i]]
            
                # Check if motor is in fault:
                if motor_mode[i] == 7:
                    
                    # If the state is not in fault we update
                    if not self.in_fault:
                        #self.rover_node.model.Elec.send_led_errors(SubSystems.HANDLING_DEVICE, Errors.FAULT.value)
                        self.in_fault = True
                else:
                    
                    # If the state was in fault we update
                    if self.in_fault:
                        if self.rover_node.rover_state_json['rover']['status']['systems']['handling_device']['status'] == 'Manual Direct':
                            pass
                            #self.rover_node.model.Elec.send_led_commands(SubSystems.HANDLING_DEVICE, 1)
                        elif self.rover_node.rover_state_json['rover']['status']['systems']['handling_device']['status'] == 'Manual Inverse':
                            pass
                            #self.rover_node.model.Elec.send_led_commands(SubSystems.HANDLING_DEVICE, 2)
                        elif self.rover_node.rover_state_json['rover']['status']['systems']['handling_device']['status'] == 'Auto':
                            pass
                            #self.rover_node.model.Elec.send_led_commands(SubSystems.HANDLING_DEVICE, 3)
                        
                        self.in_fault = False

            
            self.rover_node.rover_state_json['handling_device']['joints'][f'joint_7']['current'] = abs(round(currents[6], 1))
