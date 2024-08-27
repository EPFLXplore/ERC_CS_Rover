from custom_msg.msg import LedsCommand, Led
class Elec:
    def __init__(self, rover_node, model):
        self.rover_node = rover_node
        self.model = model

        # 0 -> nav, 1 -> hd, 2 -> drill

        self.leds = self.rover_node.node.create_publisher(LedsCommand, 
                                                             self.rover_node.el_names["/**"]["ros__parameters"]["el_pubsub_led_commands"], 1)

    def send_led_commands(self, subsystem, mode):

        match subsystem:
            case 'nav':
                self.send_mode("nav", mode, 0, 33)
            case 'hd':
                self.send_mode("hd", mode, 33, 66)
            case 'drill':
                self.send_mode("drill", mode, 66, 100)


    def send_mode(self, system, mode, low, high):
        led = Led()
        led.low = low
        led.high = high
        led.system = self.model.name_system[system]

        match mode:
            case 'Manual':
                led.mode = 0
            case 'Manual Direct':
                led.mode = 1
            case 'Manual Inverse':
                led.mode = 2
            case 'Auto':
                led.mode = 3
            case 'Off':
                led.mode = 4
            
        command = LedsCommand()
        command.leds = [led]
        self.leds.publish(command)

    def update_mass_measurement(self, msg):
        self.rover_node.rover_state_json['electronics']['sensors']['mass_sensor']["drill"] = msg.mass[1]
        # self.rover_node.rover_state_json['electronics']['sensors']['mass_sensor']["drill"] = msg.mass[1]
        # self.rover_node.rover_state_json['electronics']['sensors']['mass_sensor']["drill"] = msg.mass[1]