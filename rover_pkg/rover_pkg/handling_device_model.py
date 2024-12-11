from rclpy.action import GoalResponse
from custom_msg.action import HDManipulation
from custom_msg.srv import RequestHDGoal
import math
from custom_msg.msg import HDGoal

class HandlingDevice:
    def __init__(self, rover_node):
        self.rover_node = rover_node

        self.running = False
    
    def make_action(self, goal_handle_cs):
        self.goal_handle_cs = goal_handle_cs
        self.rover_node.node.get_logger().info("HD action starting... ")

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

    def hd_motor_cmds(self, msg):
        
        if self.rover_node.rover_state_json['rover']['status']['systems']['handling_device']['status'] == 'Off':
            # update the rover status
            for i in range(7):
                self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['angle'] = '0.0'
                self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['velocity'] = '0.0'
                self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['current'] = '0.0'
                self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['torque'] = '0.0'
                self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['state'] = False
                self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['mode_motor'] = 0

            return           
        

        currents = msg.current
        motor_mode = msg.motor_mode
        positions = msg.position
        vel = msg.velocity
        torque = msg.torque

        # This helps with the CS to have either motor connected, or disconnected
        on_off = [True if motor_mode[i] == 4 else False for i in range(len(motor_mode))]

        # update the rover status
        for i in range(7):
            self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['angle'] = round(math.degrees(positions[i]), 1)
            self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['velocity'] = vel[i]
            self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['current'] = round(currents[i], 1)
            self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['torque'] = torque[i]
            self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['state'] = on_off[i]
            self.rover_node.rover_state_json['handling_device']['joints'][f'joint_{i+1}']['mode_motor'] = motor_mode[i]