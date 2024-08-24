from rclpy.action import GoalResponse, CancelResponse
from std_msgs.msg import String
from custom_msg.action import DrillCmd

class Drill:
    def __init__(self, rover_node):
        self.rover_node = rover_node

        self.feedback = None
        self.result = None
        self.status = None
    
    '''
    Function pre-handling the request from CS. Accept or Reject
    '''
    def action_status(self, goal_handle_cs):
        if self.rover_node.rover_state_json['rover']['status']['systems']['drill']['status'] == 'Off':
            return GoalResponse.REJECT
        
        return GoalResponse.ACCEPT

    '''
    Function handling the request from CS.
    '''
    def make_action(self, goal_handle_cs):
        self.goal_handle_cs = goal_handle_cs
        print("Drill action starting... " + str(self.goal_handle_cs.request.action))

        self.result = None
        self.status = None
        self.feedback = None

        # SEND ACTION TO DRILL

        self.rover_node.drill_action_client.wait_for_server()
        future_c = self.rover_node.drill_action_client.send_goal_async(goal_handle_cs.request, 
                                self.feedback_callback)
        
        future_c.add_done_callback(self.drill_response_callback)

    '''
    Function handling the response of the request to the Drill.
    '''
    def drill_response_callback(self, future):
        self.goal_handle_drill = future.result()

        # GOAL REJECTED FROM DRILL - FORWARD TO CS (return is sufficient? need to test)

        if not self.goal_handle_drill.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        
        get_result_future = self.goal_handle_drill.get_result_async()
        get_result_future.add_done_callback(self.result_callback)
       
    '''
    Function handling the result of the action to the Drill. Return the result and the status to the CS as an object
    '''
    def result_callback(self, future):
        self.result = future.result().result # should be exactly the same format (DrillCmd)
        self.status = future.result().status
        self.feedback = None

        # EVERYTHING WENT FINE - ACCEPT

        self.goal_handle_cs.succeed()
        return self.result, self.status

    '''
    Function forwarding the feedback from Drill to CS. Handle also the cancellation from CS
    '''
    def feedback_callback(self, feedback):
        self.feedback = feedback.feedback
        self.goal_handle_cs.publish_feedback(self.feedback)

        # CANCELED THE ACTION
        if self.goal_handle_cs.is_cancel_requested:

            # CANCELATION IN DRILL ACCEPTED FROM THE DRILL - FORWARD TO CS

            self.goal_handle_cs.canceled()
            response_result = DrillCmd.Result()
            response_result.result = "action canceled successfully"
            response_result.error_type = 1
            response_result.error_message = "no error on cancellation"
            self.status = None
            return response_result

    '''
    Cancel action from CS. Need to send cancellation to DRILL and forward cancellation
    '''
    def cancel_goal_from_cs(self, goal_handle_cs):
        future_drill = self.goal_handle_drill.cancel_goal_async()
        future_drill.add_done_callback(self.cancel_drill_action)
    
    '''
    Cancel action from ROVER. DONT KNOW IF IT WILL WORK BECAUSE OF CALLBACK RETURN
    '''
    def cancel_drill_action(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
            return CancelResponse.ACCEPT
        else:
            self.get_logger().info('Goal failed to cancel')
            return CancelResponse.REJECT
        
    
    # ------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------

    def update_motor_status(self, msg):
        self.rover_node.rover_state_json['drill']['motors']['motor_module']['position'] = msg.encoder
        self.rover_node.rover_state_json['drill']['motors']['motor_drill']['speed'] = msg.vel

    def update_drill_status(self, msg):
        self.rover_node.rover_state_json['drill']['state']['current_step'] = msg.data
    