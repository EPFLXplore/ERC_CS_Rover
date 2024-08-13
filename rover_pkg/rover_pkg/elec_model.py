
class Elec:
    def __init__(self, rover_node):
        self.rover_node = rover_node

        # self.node.create_subscription(Voltage, 'EL/voltage', self.update_battery_voltage , 10)
        
    def action_status(self, goal):
        return
    
    def make_action(self, goal_handle):
        pass

    def update_mass_measurement(self, msg):
        self.rover_node.rover_state_json['electronics']['sensors']['mass_sensor']["drill"] = msg.mass[1]
        # self.rover_node.rover_state_json['electronics']['sensors']['mass_sensor']["drill"] = msg.mass[1]
        # self.rover_node.rover_state_json['electronics']['sensors']['mass_sensor']["drill"] = msg.mass[1]