from rclpy.action import GoalResponse, CancelResponse
from custom_msg.action import HDManipulation
from custom_msg.srv import RequestHDGoal
import math, time
from custom_msg.msg import HDGoal

class HandlingDevice:
    def __init__(self, rover_node):
        self.rover_node = rover_node

        # info related to handling device parameters
        # self.joint_telemetry = [0,0,0,0,0,0]
        # self.joint_command = [0,0,0,0,0,0]
        # self.joint_state = [0,0,0,0,0,0]

        # HD --> Rover
        #self.node.create_subscription(JointState, 'HD/motor_control/joint_telemetry', self.update_hd_joint_telemetry , 10)
        self.running = False
    
    def make_action(self, goal_handle_cs):
        self.goal_handle_cs = goal_handle_cs
        self.rover_node.node.get_logger().info("HD action starting... ")

        # SEND SERVICE TO HD
        goal = self.createHdGoal(goal_handle_cs.request.action)

        future = self.rover_node.hd_manipulation_service.call_async(goal)
        future.add_done_callback(lambda f: self.hd_response_callback)
        
        while self.running:
            continue

        self.rover_node.node.get_logger().info("Canceled goal hd successfull")
        return self.result_hd_action("", 0, "no errors")

    
    def action_status(self, goal):
        if self.rover_node.rover_state_json['rover']['status']['systems']['handling_device']['status'] == 'Off':
            return GoalResponse.REJECT
        
        self.running = True
        return GoalResponse.ACCEPT
    
    '''
    Function handling the response of the request to the Drill.
    '''
    def hd_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.rover_node.node.get_logger().info("HD request from ROVER good")
                self.goal_handle_cs.succeed()
                self.running = False
            else:
                pass
        except Exception as e:
            pass
    
    def createHdGoal(self, action):
        goal = HDGoal()

        if action == "home" or action == "zero" or action == "cobra":
            goal.target = "named_pose"
            goal.predefined_pose = action

        else:
            goal.target = action
        
        sent_action = RequestHDGoal.Goal()
        sent_action.goal = goal

        return sent_action
    

    def result_hd_action(self, resultt, error_type, error_messsage):
        result = HDManipulation.Result()
        result.result = resultt
        result.error_type = error_type
        result.error_message = error_messsage
        return result


    # -----------------------------------------------------------------------------

    def hd_joint_state(self, joint_state):

        self.joint_positions = joint_state.position
        self.joint_velocities = joint_state.velocity
        self.joint_current = joint_state.effort

        # update the rover status
        for i in range(6):
            self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['angle'] = math.degrees(self.joint_positions[i])
            self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['velocity'] = self.joint_velocities[i]
            self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['current'] = self.joint_current[i]
