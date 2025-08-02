from custom_msg.msg import MassPacket, FourInOne, LEDMessage, BMS, DustData
from enum import Enum
from .states import SubSystems, Errors, LedMode

'''
Author: Giovanni Ranieri
Year: 2024-25
Description: Elec Model. This class handles the elec feedback, and state management.
'''

class Elec:
    def __init__(self, rover_node):
        self.rover_node = rover_node

        '''
        Each number is a mode, for example 1 and 2 are ackermann and omnidrive
        '''
        self.modes_nav = {
            0: LedMode.OFF,
            1: LedMode.MANUAL,
            2: LedMode.MANUAL,
            3: LedMode.AUTO
        }
        
        self.modes_hd = {
            0: LedMode.OFF,
            1: LedMode.MANUAL,
            2: LedMode.MANUAL,
            3: LedMode.AUTO
        }
        
        self.modes_drill = {
            0: LedMode.OFF,
            1: LedMode.MANUAL,
            2: LedMode.AUTO,
        }

        # Pub-Sub system for electronic subsystems, with sensors and led system
        self.led_pub = self.rover_node.node.create_publisher(LEDMessage, 
                                                             self.rover_node.el_names["LED_COM_TOPIC"], 1)

        self.rover_node.node.create_subscription(MassPacket, 
                                                  self.rover_node.el_names["MASS_TOPIC"], self.mass_callback, 1)

        self.rover_node.node.create_subscription(FourInOne,
                                                    self.rover_node.el_names["FOUR_IN_ONE_TOPIC"], self.four_in_one_callback, 1)

        self.rover_node.node.create_subscription(BMS, self.rover_node.el_names['BMS_TOPIC'], self.bms_callback, 1)


        self.rover_node.node.create_subscription(DustData, self.rover_node.el_names['DUST_TOPIC'], self.dust_sensor_callback, 1)


    def reset_informations(self):
        self.rover_node.rover_state_json['electronics']['sensors']['dust_sensor']["pm1_0_std"] = "0",
        self.rover_node.rover_state_json['electronics']['sensors']['dust_sensor']["pm2_5_std"] =  "0",
        self.rover_node.rover_state_json['electronics']['sensors']['dust_sensor']["pm10_std"] =  "0",
        self.rover_node.rover_state_json['electronics']['sensors']['dust_sensor']["pm1_0_atm"] =  "0",
        self.rover_node.rover_state_json['electronics']['sensors']['dust_sensor']["pm2_5_atm"] =  "0",
        self.rover_node.rover_state_json['electronics']['sensors']['dust_sensor']["pm10_atm"] =  "0",
        self.rover_node.rover_state_json['electronics']['sensors']['dust_sensor']["num_particles_0_3"] =  "0",
        self.rover_node.rover_state_json['electronics']['sensors']['dust_sensor']["num_particles_0_5"] =  "0",
        self.rover_node.rover_state_json['electronics']['sensors']['dust_sensor']["num_particles_1_0"] =  "0",
        self.rover_node.rover_state_json['electronics']['sensors']['dust_sensor']["num_particles_2_5"] =  "0",
        self.rover_node.rover_state_json['electronics']['sensors']['dust_sensor']["num_particles_5_0"] =  "0",
        self.rover_node.rover_state_json['electronics']['sensors']['dust_sensor']["num_particles_10"] =  "0"

        self.rover_node.rover_state_json['electronics']['power'] = {
            "voltage": "0.0",
            "current": "0.0",
            "state": "NO DATA"
        }
        
        self.rover_node.rover_state_json['electronics']['sensors']['four_in_one'] = {
            "temperature": "0.0",
            "humidity": "0.0",
            "conductivity": "0.0",
            "ph": "0.0"
        }
        self.rover_node.rover_state_json['electronics']['sensors']['mass_sensors']["mass_drill"] = "0.0"
        self.rover_node.rover_state_json['electronics']['sensors']['mass_sensors']["mass_container"] = "0.0"


    '''
    Function that sends the right color to the led system.
    '''
    def send_led_commands(self, subsystem, mode):
        
        led = LEDMessage()
        led.system = subsystem.value

        match subsystem:
            case SubSystems.NAVIGATION:
                led.mode = self.modes_nav[mode].value[0]
            case SubSystems.HANDLING_DEVICE:
                led.mode = self.modes_hd[mode].value[0]
            case SubSystems.DRILL:
                led.mode = self.modes_drill[mode].value[0]
        
        self.led_pub.publish(led)
        
    '''
    Function that sends an error to the led system. If the error is the emergency
    or the reset motors, the subsystem is not needed. 
    '''
    def send_led_errors(self, subsystem, error_type):
        if error_type == Errors.EMERGENCY_SHUTDOWN:
            led = LEDMessage()
            led.mode = LedMode.EMERGENGY_SHUTDOWN.value
            self.led_pub.publish(led)
        elif error_type == Errors.RESET_MOTORS:
            led = LEDMessage()
            led.mode = LedMode.RESET_MOTORS.value
            self.led_pub.publish(led)
        elif error_type == Errors.FAULT:
            led = LEDMessage()
            led.system = subsystem.value
            led.mode = LedMode.FAULT.value
            self.led_pub.publish(led)
    
    def dust_sensor_callback(self, msg):
        self.rover_node.rover_state_json['electronics']['sensors']['dust_sensor'] = {
            "pm1_0_std": msg.pm1_0_std,
            "pm2_5_std": msg.pm2_5_std,
            "pm10_std": msg.pm10_std,
            "pm1_0_atm": msg.pm1_0_atm,
            "pm2_5_atm": msg.pm2_5_atm,
            "pm10_atm": msg.pm10_atm,
            "num_particles_0_3": msg.num_particles_0_3,
            "num_particles_0_5": msg.num_particles_0_5,
            "num_particles_1_0": msg.num_particles_1_0,
            "num_particles_2_5": msg.num_particles_2_5,
            "num_particles_5_0": msg.num_particles_5_0,
            "num_particles_10": msg.num_particles_10
        }

    def mass_callback(self, msg):
        if (msg.id == 5):
            self.rover_node.rover_state_json['electronics']['sensors']['mass_sensors']["mass_drill"] = round(msg.mass, 3)   
        elif (msg.id == 7):
            self.rover_node.rover_state_json['electronics']['sensors']['mass_sensors']["mass_container"] = round(msg.mass, 3)
        
    
    def four_in_one_callback(self, msg):
        self.rover_node.rover_state_json['electronics']['sensors']['four_in_one'] = {
            "temperature": round(msg.temperature, 1),
            "humidity": round(msg.humidity, 1),
            "conductivity": round(msg.conductivity, 0),
            "ph": round(msg.ph, 1)
        }
    
    def bms_callback(self, msg):
        self.rover_node.rover_state_json['electronics']['power'] = {
            "voltage": msg.v_bat,
            "current": msg.current,
            "state": msg.status
        }


