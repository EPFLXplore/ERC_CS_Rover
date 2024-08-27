from std_msgs.msg import String
from concurrent.futures import Future

class Drill:
    def __init__(self, rover_node):
        self.rover_node = rover_node

    '''
    Function handling the request from CS.
    
    def make_action(self, goal_handle_cs):
        self.goal_handle_cs = goal_handle_cs
        print("Drill action starting... " + str(self.goal_handle_cs.request))

        drill = DrillTerrain.Goal()
        drill.extend_to_percentage = goal_handle_cs.request.extend_to_percentage

        self.result = None
        self.feedback = None
        self.action = True
        self.canceled = False

        self.goal_handle_drill = None

        self.rover_node.drill_action_client.wait_for_server()
        print("OKOKOK")
        future_c = self.rover_node.drill_action_client.send_goal_async(drill, 
                                self.feedback_callback)
        print("B")
        future_c.add_done_callback(self.drill_response_callback)
        
        while self.action:
            continue
        
        if not self.canceled:
            response_result = DrillTerrain.Result()
            response_result.result = "action finished"
            response_result.error_type = 0
            response_result.error_message = "no error"
            return response_result
    
        else:
            print("bbbbbbbb")
            response_result = DrillTerrain.Result()
            response_result.result = "action canceled successfully"
            response_result.error_type = 1
            response_result.error_message = "no error on cancellation"
            return response_result
    
    
    Function handling the response of the request to the Drill.
    
    def drill_response_callback(self, future):
        self.goal_handle_drill = future.result()

        # GOAL REJECTED FROM DRILL - FORWARD TO CS (return is sufficient? need to test)

        if not self.goal_handle_drill.accepted:
            self.rover_node.node.get_logger().info('Goal rejected')
            return

        self.rover_node.node.get_logger().info('Goal accepted')
        
        get_result_future = self.goal_handle_drill.get_result_async()
        get_result_future.add_done_callback(self.result_callback)
       
    
    Function handling the result of the action to the Drill. Return the result and the status to the CS as an object
    
    def result_callback(self, future):
        # EVERYTHING WENT FINE - ACCEPT

        if future.result().error_type == 1:
            self.goal_handle_cs.canceled()
            print("C")
            return

        self.goal_handle_cs.succeed()
        self.result = future.result().result
        self.feedback = None
        self.action = False
        print("FINISH")

    
    Function forwarding the feedback from Drill to CS. Handle also the cancellation from CS
    
    def feedback_callback(self, feedback):
        self.feedback = feedback.feedback

        if self.feedback != None:
            self.goal_handle_cs.publish_feedback(self.feedback)

        
        # CANCELED THE ACTION
        #if self.goal_handle_cs.is_cancel_requested:

            # CANCELATION IN DRILL ACCEPTED FROM THE DRILL - FORWARD TO CS
            self.goal_handle_cs.canceled()
            response_result = DrillTerrain.Result()
            response_result.result = "action canceled successfully"
            response_result.error_type = 1
            response_result.error_message = "no error on cancellation"
            return response_result
        
            

    
    Cel action from CS. Need to send cancellation to DRILL and forward cancellation
    
    def cancel_goal_from_cs(self, goal_handle_cs):
        self.rover_node.node.get_logger().info("canceling goal in rover")
        tt = String()
        tt.data = "cancel"
        self.test.publish(tt)
        self.canceled = True
        #future_drill = self.goal_handle_drill.cancel_goal_async()
        #future_drill.add_done_callback(self.cancel_drill_action)
        
    
    Cancel action from ROVER. DONT KNOW IF IT WILL WORK BECAUSE OF CALLBACK RETURN
    
    def cancel_drill_action(self, future):
        cancel_response = future.result()
        print(cancel_response)
        if len(cancel_response.goals_canceling) > 0:
            self.rover_node.node.get_logger().info('Goal successfully canceled')
            self.result_future.set_result(DrillTerrain.Result(result="action canceled successfully", error_type=1, error_message="no error on cancellation"))
        else:
            self.rover_node.node.get_logger().info('Goal failed to cancel')
            self.result_future.set_result(DrillTerrain.Result(result="action failed to cancel", error_type=2, error_message="error on cancellation"))
    '''
    
    # ------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------

    def update_motor_status(self, msg):
        self.rover_node.rover_state_json['drill']['motors']['motor_module']['position'] = msg.encoder
        self.rover_node.rover_state_json['drill']['motors']['motor_drill']['speed'] = msg.vel

    def update_drill_status(self, msg):
        self.rover_node.rover_state_json['drill']['state']['current_step'] = msg.data
    