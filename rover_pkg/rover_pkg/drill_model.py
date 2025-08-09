from rclpy.action import GoalResponse, CancelResponse
from std_msgs.msg import Bool
from custom_msg.action import DrillCmd
from .states import SubSystems, LedMode

'''
Author: Giovanni Ranieri
Year: 2024-25
Description: Drill Model. This class handles the drill actions, feedback, and state management.
'''

class Drill:
    def __init__(self, rover_node):
        self.rover_node = rover_node

        # List of action for the drill. This list has to be the same as the one in the Drill FSM project.
        self.modes = {
            0: 'STOPPED',
            1: 'IDLE', 
            2: 'DRILLSTART',
            3: 'EXTEND',
            4: 'RETURN',
            5: 'RELEASE', 
            6: 'OPEN',
            7: 'CLOSE',
            8: 'SEMI_RETURN',
            9: 'STEP_DOWN',
            10: 'STEP_UP'
        }

        # Standard variables
        self.feedback = None
        self.running = False
        self.cancel_drill = False
        self.result = None
        self.counter_cancel = 0
        self.in_fault = False

        # Subscription for the subsystem state
        self.rover_node.node.create_subscription(Bool, self.rover_node.science_names['status_system'], self.handle_state, 10)
    
    def reset_informations(self):
        if not self.rover_node.emergency_state:
            self.rover_node.model.Elec.send_led_commands(SubSystems.DRILL, LedMode.OFF)

        self.rover_node.rover_state_json['drill']['motors']['motor_module']['position'] = "0.0"
        self.rover_node.rover_state_json['drill']['motors']['motor_drill']['speed'] = "0.0"
        self.rover_node.rover_state_json['drill']['motors']['motor_module']['current'] = "0.0"
        self.rover_node.rover_state_json['drill']['motors']['motor_drill']['current'] = "0.0"
        self.rover_node.rover_state_json['drill']['motors']['motor_drill']['state'] = False
        self.rover_node.rover_state_json['drill']['motors']['motor_module']['state'] = False

    '''
    Function handling the state of the drill's subsystem. 
    If the state is 1, it means the drill is on, otherwise it is off.
    '''
    def handle_state(self, msg):
        if msg.data == 1:
            self.rover_node.rover_state_json['rover']['status']['systems']['drill']['status'] = 'On'
        else:
            self.rover_node.rover_state_json['rover']['status']['systems']['drill']['status'] = 'Off'
            self.reset_informations()

    '''
    Function pre-handling the request from CS. Accept or Reject
    '''
    def action_status(self, goal_handle_cs):
        if self.rover_node.rover_state_json['rover']['status']['systems']['drill']['status'] == 'Off':
            return GoalResponse.REJECT
        
        self.result = None
        self.running = True
        self.feedback = None
        self.cancel_drill = False
        self.counter_cancel = 0
        return GoalResponse.ACCEPT

    '''
    Function handling the request from CS.
    It forwards the request to the drill action server and waits for the result.
    '''
    def make_action(self, goal_handle_cs):
        self.goal_handle_cs = goal_handle_cs
        self.rover_node.node.get_logger().info("Drill action starting... ")

        self.rover_node.drill_action_client.wait_for_server()
        future_c = self.rover_node.drill_action_client.send_goal_async(self.goal_handle_cs.request, 
                                self.feedback_callback)
        
        future_c.add_done_callback(self.drill_response_callback)
        
        # Hard wait until the drill action is finished
        while self.running:
            continue

        if not self.cancel_drill:
            self.rover_node.node.get_logger().info("Drill Goal finished successfully")
            return self.result_drill_action(self.result)
        else:
            self.rover_node.node.get_logger().info("Canceled goal drill successfull")
            return self.cancel_result()

    '''
    Function handling the response of the request to the Drill.
    '''
    def drill_response_callback(self, future):
        self.goal_handle_drill = future.result()

        # If the drill rejects the goal, we cancel the overall action, else we accept it.
        if not self.goal_handle_drill.accepted:
            self.cancel_drill = True
            self.running = False
            self.rover_node.node.get_logger().info('Drill Goal rejected from drill')
            return self.result_drill_action(self.result)

        self.rover_node.node.get_logger().info('Drill Goal accepted from drill')
        
        get_result_future = self.goal_handle_drill.get_result_async()
        get_result_future.add_done_callback(self.result_callback)
       
    '''
    Function handling the result of the action to the Drill. Return the result and the status to the CS as an object
    '''
    def result_callback(self, future):
        self.result = future.result().result
        self.feedback = None

        if not self.cancel_drill:
            self.goal_handle_cs.succeed()

        self.running = False

    '''
    Function forwarding the feedback from Drill to CS. Handle also the cancellation from CS
    '''
    def feedback_callback(self, feedback):

        # If the action is canceled, we need to cancel the goal drill
        if self.cancel_drill and self.counter_cancel == 0:
            self.counter_cancel = self.counter_cancel + 1
            future_drill = self.goal_handle_drill.cancel_goal_async()
            future_drill.add_done_callback(self.cancel_drill_action)
        
        # else we just update the feedback
        else:
            self.feedback = feedback.feedback

    '''
    Cancel action from CS. Need to send cancellation to DRILL and forward cancellation
    '''
    def cancel_goal_from_cs(self, goal_handle_cs):
        self.rover_node.node.get_logger().info("Drill goal cancelation requested...")
        self.cancel_drill = True

        return CancelResponse.ACCEPT
    
    '''
    Cancel action from ROVER.
    When the cancel action from CS is called, this callback is triggered when the drill action server
    sends the cancel response.
    It checks if the cancel response is successful and updates the rover state JSON accordingly.
    If the cancel is successful, it sets the action from CS to canceled and stops the running state
    '''
    def cancel_drill_action(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.rover_node.node.get_logger().info('Drill Goal successfully canceled')
        
            # NEWWWW
            self.goal_handle_cs.canceled()
            self.running = False
        else:
            self.rover_node.node.get_logger().error('Drill Goal failed to cancel...')
            # if enter here.. bad for us
        
    
    # ------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------

    '''
    Function callback for the drill motors status.
    It updates the drill state in the rover state JSON.
    '''
    def update_motor_status(self, msg):
        
        if self.rover_node.rover_state_json['rover']['status']['systems']['drill']['status'] == 'Off':
            return

        self.rover_node.rover_state_json['drill']['motors']['motor_module']['position'] = round(msg.distance)
        self.rover_node.rover_state_json['drill']['motors']['motor_drill']['speed'] = msg.vel
        self.rover_node.rover_state_json['drill']['motors']['motor_module']['current'] = abs(msg.trans_current)
        self.rover_node.rover_state_json['drill']['motors']['motor_drill']['current'] = abs(msg.screw_current)
        self.rover_node.rover_state_json['drill']['motors']['motor_drill']['state'] = msg.motor_screw
        self.rover_node.rover_state_json['drill']['motors']['motor_module']['state'] = msg.motor_trans
        
        # TODO NEXT YEAR, ADD FAULT IN CUSTOM MESSAGE SORRY
        # if msg.motor_trans or msg.motor_screw:
        #     if not self.in_fault:
        #         self.rover_node.model.Elec.send_led_commands(SubSystems.DRILL, LedMode.FAULT)
        #         self.in_fault = True
        # else:
        #     if self.in_fault: 
        #         self.rover_node.model.Elec.send_led_commands(SubSystems.DRILL, LedMode.MANUAL)
        #         self.in_fault = False


    '''
    Function handling the drill FSM state.
    It could be directly integrated in the drill status feedback update_motor_status(), becoming
    a more generic function.
    '''
    def update_drill_status(self, msg):
        if self.rover_node.rover_state_json['rover']['status']['systems']['drill']['status'] == 'Off':
            self.rover_node.rover_state_json['drill']['state']['state_fsm'] = 'IDLE'
            return

        self.rover_node.rover_state_json['drill']['state']['state_fsm'] = self.modes[msg.mode]


    '''
    Utility function
    '''
    def result_drill_action(self, resultt):
        result = DrillCmd.Result()
        result.result = resultt.result
        result.error_type = resultt.error_type
        result.error_message = resultt.error_message
        return result
    
    def cancel_result(self):
        result = DrillCmd.Result()
        result.result = "Action Drill has been canceled"
        result.error_type = 1
        result.error_message = "Stopped by operator"
        return result
    