from custom_msg.msg import LedCommands, Leds
class Elec:
    def __init__(self, rover_node):
        self.rover_node = rover_node

        self.leds = self.rover_node.node.create_publisher(Leds, 
                                                             self.rover_node.el_names["ros__parameters"]["el_pubsub_led_commands"], 1)

    def send_led_commands(self, subsystem, mode):

        match mode:
            case 'Manual':
                self.blink_manual(subsystem, 0)
            case 'Manual Direct':
                self.blink_manual(subsystem, 0)
            case 'Manual Inverse':
                self.blink_manual(subsystem, 2)
            case 'Auto':
                self.blink_auto(subsystem)
            case 'Off':
                self.stop(subsystem)

    '''
    In Manual mode: no blink, just the color of the subsystem
    '''
    def blink_manual(self, subsystem, blink):
        match subsystem:
            case "nav":
                command = self.purple()
                command.low = 0
                command.high = 33
            case "hd":
                command = self.yellow()
                command.low = 33
                command.high = 66
            case "drill":
                command = self.marron()
                command.low = 66
                command.high = 100
        
        command.time_blink = blink
        leds = Leds()
        leds.leds[0] = command
        self.leds.publish(leds)

    '''
    In Auto mode: blink 3 times in 5s, the overall 3 times
    '''
    def blink_auto(self, subsystem):
        match subsystem:
            case "nav":
                command = self.purple()
                command.low = 0
                command.high = 33

            case "hd":
                command = self.yellow()
                command.low = 33
                command.high = 66
                
            case "drill":
                command = self.marron()
                command.low = 66
                command.high = 100
                        
        command.time_blink = 3
        leds = Leds()
        leds.leds[0] = command
        self.leds.publish(leds)

    '''
    
    '''
    def blue(self):
        command = LedCommands()
        command.red = 0
        command.green = 0
        command.blue = 255

        return command
    
    def purple(self):
        command = LedCommands()
        command.red = 102
        command.green = 0
        command.blue = 204

        return command
    
    def yellow(self):
        command = LedCommands()
        command.red = 255
        command.green = 255
        command.blue = 0

        return command
    
    def marron(self):
        command = LedCommands()
        command.time_blink = 0
        command.red = 204
        command.green = 102
        command.blue = 0

        return command
    
    def stop(self, subsystem):
        command = self.blue()
        match subsystem:
            case "nav":
                command.low = 0
                command.high = 33

            case "hd":
                command.low = 33
                command.high = 66
                
            case "drill":
                command.low = 66
                command.high = 100
                        
        command.time_blink = 0
        leds = Leds()
        leds.leds[0] = command
        self.leds.publish(leds)

    def update_mass_measurement(self, msg):
        self.rover_node.rover_state_json['electronics']['sensors']['mass_sensor']["drill"] = msg.mass[1]
        # self.rover_node.rover_state_json['electronics']['sensors']['mass_sensor']["drill"] = msg.mass[1]
        # self.rover_node.rover_state_json['electronics']['sensors']['mass_sensor']["drill"] = msg.mass[1]