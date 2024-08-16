from rclpy.action import GoalResponse, CancelResponse
from custom_msg.action import HDManipulation
import math, time

class HandlingDevice:
    def __init__(self, rover_node):
        self.rover_node = rover_node

        # info related to handling device parameters
        # self.joint_telemetry = [0,0,0,0,0,0]
        # self.joint_command = [0,0,0,0,0,0]
        # self.joint_state = [0,0,0,0,0,0]

        # HD --> Rover
        #self.node.create_subscription(JointState, 'HD/motor_control/joint_telemetry', self.update_hd_joint_telemetry , 10)

    def hd_joint_state(self, joint_state):

        self.joint_positions = joint_state.position
        self.joint_velocities = joint_state.velocity
        self.joint_current = joint_state.effort

        # update the rover status
        for i in range(7):
            self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['angle'] = math.degrees(self.joint_positions[i])
            self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['velocity'] = self.joint_velocities[i]
            self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['current'] = self.joint_current[i]
    
    def make_action(self, goal_handle):
        print("Manipulation HD action starting...")  
        print("Goal type: " + str(goal_handle.request.task_type) + " Goal id: " + str(goal_handle.request.task_id) )

        feedback = HDManipulation.Feedback()
        i = 0

        while i < 10:
            time.sleep(10)
            feedback.current_status = "ok"
            feedback.warning_type = 0
            feedback.warning_message = "no warning"
            goal_handle.publish_feedback(feedback)
            print("feedback running hd")
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return HDManipulation.Result()
            i = i + 1
        
        goal_handle.succeed()

        print("Manipulation HD action is finished")
        result = HDManipulation.Result()
        result.result = "action finished"
        result.error_type = 0
        result.error_message = "no error"
        return result
    
    def action_status(self, goal):
        if self.rover_node.rover_state_json['rover']['status']['systems']['handling_device']['status'] == 'Off':
            return GoalResponse.REJECT
        
        return GoalResponse.ACCEPT
    
    def cancel_goal(self, goal):
        print("cancel goal hd")
        return CancelResponse.ACCEPT