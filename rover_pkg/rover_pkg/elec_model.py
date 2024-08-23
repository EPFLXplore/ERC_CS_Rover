from custom_msg.msg import LedsCommand, Led
class Elec:
    def __init__(self, rover_node):
        self.rover_node = rover_node

        self.leds = self.rover_node.node.create_publisher(LedsCommand, 
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
                led = self.purple()
                led.low = 0
                led.high = 33
            case "hd":
                led = self.yellow()
                led.low = 33
                led.high = 66
            case "drill":
                led = self.marron()
                led.low = 66
                led.high = 100
        
        led.time_blink = blink
        command = LedsCommand()
        command.leds = [led]
        self.leds.publish(command)

    '''
    In Auto mode: blink 3 times in 5s, the overall 3 times
    '''
    def blink_auto(self, subsystem):
        match subsystem:
            case "nav":
                led = self.purple()
                led.low = 0
                led.high = 33

            case "hd":
                led = self.yellow()
                led.low = 33
                led.high = 66
                
            case "drill":
                led = self.marron()
                led.low = 66
                led.high = 100
                        
        led.time_blink = 3
        command = LedsCommand()
        command.leds = [led]
        self.leds.publish(command)

    '''
    
    '''
    def blue(self):
        led = Led()
        led.red = 0
        led.green = 0
        led.blue = 255

        return led
    
    def purple(self):
        led = Led()
        led.red = 102
        led.green = 0
        led.blue = 204

        return led
    
    def yellow(self):
        led = Led()
        led.red = 255
        led.green = 255
        led.blue = 0

        return led
    
    def marron(self):
        led = Led()
        led.time_blink = 0
        led.red = 204
        led.green = 102
        led.blue = 0

        return led
    
    def stop(self, subsystem):
        led = self.blue()
        match subsystem:
            case "nav":
                led.low = 0
                led.high = 33

            case "hd":
                led.low = 33
                led.high = 66
                
            case "drill":
                led.low = 66
                led.high = 100
                        
        led.time_blink = 0
        command = LedsCommand()
        command.leds = [led]
        self.leds.publish(command)

    def update_mass_measurement(self, msg):
        self.rover_node.rover_state_json['electronics']['sensors']['mass_sensor']["drill"] = msg.mass[1]
        # self.rover_node.rover_state_json['electronics']['sensors']['mass_sensor']["drill"] = msg.mass[1]
        # self.rover_node.rover_state_json['electronics']['sensors']['mass_sensor']["drill"] = msg.mass[1]