from rclpy.action import GoalResponse, CancelResponse
from std_msgs.msg import Bool
from custom_msg.action import DrillCmd
from .states import SubSystems, Errors, LedMode

class Drill:
    def __init__(self, rover_node):
        self.rover_node = rover_node
        
        self.in_fault = False

        self.modes = {
            0: 'STOPPED',
            1: 'IDLE', 
            2: 'DRILLSTART',
            3: 'EXTEND',
            4: 'RETURN',
            5: 'RELEASE',
            6: 'OPEN',
            7: 'CLOSE',
            8: 'WAIT',
            9: 'SEMI_RETURN'
        }

        self.feedback = None
        self.running = False
        self.cancel_drill = False
        self.result = None
        self.counter_cancel = 0

        self.rover_node.node.create_subscription(Bool, self.rover_node.science_names['status_system'], self.handle_state, 10)
    
    def reset_informations(self):
        self.rover_node.model.Elec.send_led_commands(SubSystems.DRILL, 0)

        self.rover_node.rover_state_json['drill']['motors']['motor_module']['position'] = "0.0"
        self.rover_node.rover_state_json['drill']['motors']['motor_drill']['speed'] = "0.0"
        self.rover_node.rover_state_json['drill']['motors']['motor_module']['current'] = "0.0"
        self.rover_node.rover_state_json['drill']['motors']['motor_drill']['current'] = "0.0"
        self.rover_node.rover_state_json['drill']['motors']['motor_drill']['state'] = False
        self.rover_node.rover_state_json['drill']['motors']['motor_module']['state'] = False

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
    '''
    def make_action(self, goal_handle_cs):
        self.goal_handle_cs = goal_handle_cs
        self.rover_node.node.get_logger().info("Drill action starting... ")

        # SEND ACTION TO DRILL

        self.rover_node.drill_action_client.wait_for_server()
        future_c = self.rover_node.drill_action_client.send_goal_async(self.goal_handle_cs.request, 
                                self.feedback_callback)
        
        future_c.add_done_callback(self.drill_response_callback)
        
        while self.running:
            continue

        if not self.cancel_drill:
            self.rover_node.node.get_logger().info("Drill Goal finished successfully")
        else:
            self.rover_node.node.get_logger().info("Canceled goal drill successfull")
        
        return self.result_drill_action(self.result)

    '''
    Function handling the response of the request to the Drill.
    '''
    def drill_response_callback(self, future):
        self.goal_handle_drill = future.result()

        # GOAL REJECTED FROM DRILL - FORWARD TO CS (return is sufficient? need to test)

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

        if self.cancel_drill and self.counter_cancel == 0:
            self.counter_cancel = self.counter_cancel + 1
            future_drill = self.goal_handle_drill.cancel_goal_async()
            future_drill.add_done_callback(self.cancel_drill_action)
        
        else:
            self.feedback = feedback.feedback
            self.update_drill_feedback(self.feedback)

    '''
    Cancel action from CS. Need to send cancellation to DRILL and forward cancellation
    '''
    def cancel_goal_from_cs(self, goal_handle_cs):
        self.rover_node.node.get_logger().info("Drill goal cancelation requested...")
        self.cancel_drill = True

        return CancelResponse.ACCEPT
    
    '''
    Cancel action from ROVER.
    '''
    def cancel_drill_action(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.rover_node.node.get_logger().info('Drill Goal successfully canceled')
        
            # TODO DO LIKE HD!!
        else:
            self.rover_node.node.get_logger().error('Drill Goal failed to cancel...')
            # if enter here.. bad for us
        
    
    # ------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------

    def update_motor_status(self, msg):
        
        if self.rover_node.rover_state_json['rover']['status']['systems']['drill']['status'] == 'Off':
            return

        self.rover_node.rover_state_json['drill']['motors']['motor_module']['position'] = round(msg.distance)
        self.rover_node.rover_state_json['drill']['motors']['motor_drill']['speed'] = msg.vel
        self.rover_node.rover_state_json['drill']['motors']['motor_module']['current'] = abs(msg.trans_current)
        self.rover_node.rover_state_json['drill']['motors']['motor_drill']['current'] = abs(msg.screw_current)
        self.rover_node.rover_state_json['drill']['motors']['motor_drill']['state'] = msg.motor_screw
        self.rover_node.rover_state_json['drill']['motors']['motor_module']['state'] = msg.motor_trans
        
        if msg.motor_trans or msg.motor_screw:
            if not self.in_fault:
                self.rover_node.model.Elec.send_led_errors(SubSystems.DRILL, Errors.FAULT)
                self.in_fault = True
        else:
            if self.in_fault: 
                self.rover_node.model.Elec.send_led_commands(SubSystems.DRILL, 1)
                self.in_fault = False


    def update_drill_status(self, msg):
        if self.rover_node.rover_state_json['rover']['status']['systems']['drill']['status'] == 'Off':
            self.rover_node.rover_state_json['drill']['state']['state_fsm'] = 'IDLE'
            return

        self.rover_node.rover_state_json['drill']['state']['state_fsm'] = self.modes[msg.mode]

    def update_drill_feedback(self, feedback): 
        self.rover_node.rover_state_json['drill']['state']['current_status'] = feedback.current_status
        self.rover_node.rover_state_json['drill']['state']['warning_type'] = feedback.warning_type


    def result_drill_action(self, resultt):
        result = DrillCmd.Result()
        self.rover_node.rover_state_json['drill']['state']['current_status'] = resultt.result
        self.rover_node.rover_state_json['drill']['state']['warning_type'] = resultt.error_type
        return result
    