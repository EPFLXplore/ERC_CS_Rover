from custom_msg.action import HDManipulation, NewHDGoal
from .states import SubSystems, Errors, LedMode
import math
from custom_msg.msg import HDGoal
from std_msgs.msg import String
from rclpy.action import GoalResponse, CancelResponse

'''
Author: Giovanni Ranieri
Year: 2024-25
Description: Handling Device Model. This class handles the HDS actions, feedback, and state management.
'''

class HandlingDevice:
    def __init__(self, rover_node):
        self.rover_node = rover_node
        
        # Standard variables
        self.in_fault = False
        self.running = False
        self.feedback = None
        self.cancel_hd = False
        self.result = None
        self.counter_cancel = 0

        ## ---------------------------
        # List of action names for the Handling Device, stored in the HDGoal message.
        ## ---------------------------
        
        # Small Switches
        self.switches = [value for key, value in vars(HDGoal).items() if key.startswith("BUTTON")]
        
        # Big Rotation Switches
        self.big_rotation_switches = [value for key, value in vars(HDGoal).items() if key.startswith("BIG_ROTATION_BUTTON")]
        
        # Small Rotation Switches
        self.small_rotation_switches = [value for key, value in vars(HDGoal).items() if key.startswith("SMALL_ROTATION_BUTTON")]
        
        # Predefined poses
        self.predefined_poses = [HDGoal.FRONT_PANEL, HDGoal.RANGEMENT, HDGoal.HOME, HDGoal.ZERO, 
                                 HDGoal.COBRA, HDGoal.ABOVE_GROUND]
        # Tools
        self.tools = [HDGoal.CLAM_TOOL]
        
        # Small Tasks
        self.small_tasks = [HDGoal.TURN_JSIX_POS_3, HDGoal.TURN_JSIX_POS_4, HDGoal.TURN_JSIX_POS_9,
                            HDGoal.TURN_JSIX_NEG_3, HDGoal.TURN_JSIX_NEG_4, HDGoal.TURN_JSIX_NEG_9]
        
        # Models
        self.models_elements = [HDGoal.MODEL_BIG_ROTATION_BUTTON_1, HDGoal.MODEL_BIG_ROTATION_BUTTON_2, HDGoal.MODEL_SMALL_ROTATION_BUTTON_1, 
                                HDGoal.MODEL_SMALL_ROTATION_BUTTON_2, HDGoal.MODEL_SMALL_ROTATION_BUTTON_3, HDGoal.MODEL_SMALL_ROTATION_BUTTON_4, 
                                HDGoal.MODEL_SMALL_ROTATION_BUTTON_5]
        
        # Subscription for the subsystem state
        self.rover_node.node.create_subscription(String, self.rover_node.hd_names['system_status'], self.handle_state, 10)

    def reset_informations(self):
        #self.rover_node.model.Elec.send_led_commands(SubSystems.HANDLING_DEVICE, 0)
        self.rover_node.rover_state_json['handling_device']['state']['current_command'] = "NONE"
        self.rover_node.rover_state_json['handling_device']['state']['task'] = "NONE" 
        self.rover_node.rover_state_json['cameras']['handling_device']['Gripper']['status'] = False
        self.rover_node.rover_state_json['cameras']['handling_device']['Gripper']['node'] = False
        self.rover_node.rover_state_json['cameras']['handling_device']['Gripper']['depth'] = False
        for i in range(7):
                self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['angle'] = '0.0'
                self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['velocity'] = '0.0'
                self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['current'] = '0.0'
                self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['torque'] = '0.0'
                self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['state'] = False
                self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['mode_motor'] = "NotReadyToSwitchOn"
                
        
        # Jetson Stats
        self.rover_node.rover_state_json['rover']['hardware']['stats_hd']['ram'] = 0
        self.rover_node.rover_state_json['rover']['hardware']['stats_hd']['load_gpu'] = 0
        self.rover_node.rover_state_json['rover']['hardware']['stats_hd']['fan_rpm'] = 0
        self.rover_node.rover_state_json['rover']['hardware']['stats_hd']['power_tot'] = 0
        self.rover_node.rover_state_json['rover']['hardware']['stats_hd']['temp_cpu'] = 0
        self.rover_node.rover_state_json['rover']['hardware']['stats_hd']['temp_gpu'] = 0
        self.rover_node.rover_state_json['rover']['hardware']['stats_hd']['utilization_cpus'] = [0, 0, 0, 0, 0, 0, 0, 0]
    
    '''
    Function handling the request from CS.
    It forwards the request to the HD action server and waits for the result.
    HD is special because we can send multiple actions at the same time.
    '''
    def make_action(self, goal_handle_cs):
        self.goal_handle_cs = goal_handle_cs
        self.rover_node.node.get_logger().info("HD action starting... ")
        
        # Create all goals to be run sequentially
        actions = goal_handle_cs.request.actions
        goals = NewHDGoal.Goal()
        array_goals = []
        
        for action in actions:
            goal = self.createHdGoal(action)
            array_goals.append(goal)
        
        goals.goals = array_goals

        self.rover_node.hd_action_client.wait_for_server()
        future_c = self.rover_node.hd_action_client.send_goal_async(goals, 
                                self.feedback_callback)
        
        future_c.add_done_callback(self.hd_response_callback)
        self.running = True
        
        # Hard wait until the HD action is finished
        while self.running:
            continue

        if not self.cancel_hd:
            self.rover_node.node.get_logger().info('FINISHED')
            return self.result_hd_action(self.result)
        else:
            self.rover_node.node.get_logger().info('CANCELED')
            return self.no_result()
    
    '''
    Function forwarding the feedback from HD to CS. Handle also the cancellation from CS
    '''
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
    

    '''
    Function pre-handling the request from CS. Accept or Reject
    '''
    def action_status(self, goal):
        if self.rover_node.rover_state_json['rover']['status']['systems']['handling_device']['status'] != 'Auto':
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
    
    '''
    Cancel action from ROVER.
    When the cancel action from CS is called, this callback is triggered when HD action server
    sends the cancel response.
    It checks if the cancel response is successful and updates the rover state JSON accordingly.
    If the cancel is successful, it sets the action from CS to canceled and stops the running state
    '''
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
    Function handling the response of the request to HD.
    '''
    def hd_response_callback(self, future):
        self.goal_handle_hd = future.result()

        # If HD rejects the goal, we cancel the overall action, else we accept it.
        if not self.goal_handle_hd.accepted:
            self.cancel_hd = True
            self.running = False
            self.rover_node.node.get_logger().info('HD Goal rejected from HD')
            return self.no_result()

        self.rover_node.node.get_logger().info('HD Goal accepted from HD')
        
        get_result_future = self.goal_handle_hd.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    
    '''
    Function creating a goal element for 1 action. 
    It checks which action it is and create the message
    '''
    def createHdGoal(self, action):
        msg_goal = HDGoal()            
            
        # Predefined poses
        if action in self.predefined_poses:
            msg_goal.target = HDGoal.NAMED_POSE
            msg_goal.predefined_pose = action

        # Tools
        elif action in self.tools:
            msg_goal.target = HDGoal.TOOL_PICKUP
            msg_goal.tool = action
            
        # Small Switches
        elif action in self.switches:
            msg_goal.target = HDGoal.BUTTON_TASK
            msg_goal.switch_name = action
            
        # Big Rotation switch
        elif action in self.big_rotation_switches:
            msg_goal.target = HDGoal.ROTATION_BUTTON_TASK
            msg_goal.maintenance_objects = [action]
            
        # Small Rotation switch
        elif action in self.small_rotation_switches:
            msg_goal.target = HDGoal.ROTATION_BUTTON_TASK
            msg_goal.maintenance_objects = [action]
            
        # Direct Approach to switch with model
        elif action in self.models_elements:
            msg_goal.target = HDGoal.MODEL_ROTATION_BUTTON_TASK
            msg_goal.model_element = action
            
        # Small Tasks
        elif action in self.small_tasks:
            msg_goal.target = HDGoal.TURN_J6
            msg_goal.rotation_degree = int(action[8:10]) # Retrieve the 30, 45 or 90
            msg_goal.clockwise_or_not = action[11:] # Retrieve the pos or neg orientation
        else:
            msg_goal.target = action

        return msg_goal
    
    '''
    Function handling the result of the action to HD. Return the result and the status to the CS as an object
    '''
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

    '''
    Function handling the state of the HD's subsystem. 
    If the state is 1, it means HD is on, otherwise it is off.
    '''
    def handle_state(self, msg):
        self.rover_node.rover_state_json['rover']['status']['systems']['handling_device']['status'] = msg.data

        if msg.data == 'Off' and self.rover_node.rover_state_json['rover']['status']['systems']['handling_device']['status'] != 'Off':
           self.reset_informations()

    '''
    Function callback for HD motors status.
    It updates HD state in the rover state JSON.
    '''
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
