from rclpy.action import GoalResponse
from std_msgs.msg import Bool
from custom_msg.action import DrillCmd

class Drill:
    def __init__(self, rover_node):
        self.rover_node = rover_node

        self.feedback = None
        self.running = False
        self.cancel_drill = False
        self.result = None
        self.counter_cancel = 0

        self.rover_node.node.create_subscription(Bool, self.rover_node.science_names["/**"]["ros__parameters"]['status_system'], self.handle_state, 10)
    
    def handle_state(self, msg):
        if msg.data:
            self.rover_node.rover_state_json['rover']['status']['systems']['drill']['status'] = 'On'
        else:
            self.rover_node.rover_state_json['rover']['status']['systems']['drill']['status'] = 'Off'
            self.rover_node.model.Elec.send_led_commands("drill", "Off")

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
            return self.result_drill_action("Drill Goal finished successfully", 0, "no errors")
        else:
            self.rover_node.node.get_logger().info("Canceled goal drill successfull")
            return self.result_drill_action(self.result.result, self.result.error_type, self.result.error_message)

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
            return self.result_drill_action("Drill Goal rejected from drill", 1, 'no errors')

        self.rover_node.node.get_logger().info('Drill Goal accepted from drill')

        self.rover_node.model.Elec.send_led_commands("drill", "action")
        
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
        self.rover_node.model.Elec.send_led_commands("drill", "On")        

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
            self.goal_handle_cs.publish_feedback(self.feedback)

    '''
    Cancel action from CS. Need to send cancellation to DRILL and forward cancellation
    '''
    def cancel_goal_from_cs(self, goal_handle_cs):
        self.rover_node.node.get_logger().info("Drill goal cancelation requested...")
        self.cancel_drill = True


    
    '''
    Cancel action from ROVER. DONT KNOW IF IT WILL WORK BECAUSE OF CALLBACK RETURN
    '''
    def cancel_drill_action(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.rover_node.node.get_logger().info('Drill Goal successfully canceled')
        else:
            self.rover_node.node.get_logger().error('Drill Goal failed to cancel...')
            # if enter here.. bad for us
        
    
    # ------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------

    def update_motor_status(self, msg):
        self.rover_node.rover_state_json['drill']['motors']['motor_module']['position'] = msg.encoder
        self.rover_node.rover_state_json['drill']['motors']['motor_drill']['speed'] = msg.vel

    def update_drill_status(self, msg):
        self.rover_node.rover_state_json['drill']['state']['current_step'] = msg.data

    def result_drill_action(self, resultt, error_type, error_messsage):
        result = DrillCmd.Result()
        result.result = resultt
        result.error_type = error_type
        result.error_message = error_messsage
        return result
    