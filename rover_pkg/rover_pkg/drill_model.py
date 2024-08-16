from rclpy.action import GoalResponse, CancelResponse
from std_msgs.msg import String
from custom_msg.action import DrillCmd

class Drill:
    def __init__(self, rover_node):
        self.rover_node = rover_node
    
    def action_status(self, goal):
        if self.rover_node.rover_state_json['rover']['status']['systems']['drill']['status'] == 'Off':
            return GoalResponse.REJECT
        
        return GoalResponse.ACCEPT

    def make_action(self, goal_handle):
        print("Drill action starting... " + str(goal_handle.request.action))

        if(goal_handle.request.action == "auto"):
            feedback = DrillCmd.Feedback()
            i = 0

            while i < 2:

                feedback.current_status = "ok"
                feedback.warning_type = 0
                feedback.warning_message = ""
                goal_handle.publish_feedback(feedback)
                print("feedback running drill")
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return DrillCmd.Result()
                i = i + 1
            
            goal_handle.succeed()
            
        else:
            action = String()
            action.data = goal_handle.request.action
            self.rover_node.sc_cmd_pub.publish(action)
            goal_handle.succeed()


        print("DRILL action is finished")
        result = DrillCmd.Result()
        result.result = ""
        result.error_type = 0
        result.error_message = ""
        return result

    def update_motor_status(self, msg):
        self.rover_node.rover_state_json['drill']['motors']['motor_module']['position'] = msg.encoder
        self.rover_node.rover_state_json['drill']['motors']['motor_drill']['speed'] = msg.vel

    def update_drill_status(self, msg):
        self.rover_node.rover_state_json['drill']['state']['current_step'] = msg.data
    
    def cancel_goal(self, goal):
        print("cancel goal drill")
        return CancelResponse.ACCEPT