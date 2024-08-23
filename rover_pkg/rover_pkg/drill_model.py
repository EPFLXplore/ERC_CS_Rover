from rclpy.action import GoalResponse, CancelResponse
from std_msgs.msg import String
from custom_msg.action import DrillCmd

class Drill:
    def __init__(self, rover_node):
        self.rover_node = rover_node

        self.feedback = None
    
    def action_status(self, goal):
        if self.rover_node.rover_state_json['rover']['status']['systems']['drill']['status'] == 'Off':
            return GoalResponse.REJECT
        
        return GoalResponse.ACCEPT

    def make_action(self, goal_handle_cs):
        print("Drill action starting... " + str(goal_handle_cs.request.action))

        self.rover_node.drill_action_client.wait_for_server()
        future = self.rover_node.drill_action_client.send_goal_async(goal_handle_cs.request, 
                                                                     lambda f: self.feedback_callback(f))
        
        # FORWARD THE FEEDBACK
        if self.feedback != None:
            goal_handle_cs.publish_feedback(self.feedback)
        
        future.add_done_callback(lambda f: self.rover_node.drill_action_client.drill_response_callback(f, goal_handle_cs))

        


    def drill_response_callback(self, future, goal_handle_cs):
        goal_handle_drill = future.result()

        # CANCEL THE ACTION
        if goal_handle_cs.is_cancel_requested:
            goal_handle_cs.canceled()
            return DrillCmd.Result()
        
        # server rover send final success of the action
        goal_handle_cs.succeed()

        result = DrillCmd.Result()
        result.result = ""
        result.error_type = 0
        result.error_message = "No errors"
        return result
    
    def feedback_callback(self, feedback):
        self.feedback = feedback

    def update_motor_status(self, msg):
        self.rover_node.rover_state_json['drill']['motors']['motor_module']['position'] = msg.encoder
        self.rover_node.rover_state_json['drill']['motors']['motor_drill']['speed'] = msg.vel

    def update_drill_status(self, msg):
        self.rover_node.rover_state_json['drill']['state']['current_step'] = msg.data
    
    def cancel_goal(self, goal):
        print("cancel goal drill")
        return CancelResponse.ACCEPT