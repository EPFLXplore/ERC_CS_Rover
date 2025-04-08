from custom_msg.msg import MassArray, FourInOne, LEDMessage, BMS, DustData

class Elec:
    def __init__(self, rover_node, model):
        self.rover_node = rover_node
        self.model = model

        # 0 -> nav, 1 -> hd, 2 -> drill

        # self.led_pub = self.rover_node.node.create_publisher(LEDMessage, 
        #                                                      self.rover_node.el_names["LED_COM_TOPIC"], 1)

        # self.rover_node.node.create_subscription(MassArray, 
        #                                           self.rover_node.el_names["DRILL_MASS_TOPIC"], self.drill_mass_callback, 1)

        # self.rover_node.node.create_subscription(MassArray,
        #                                             self.rover_node.el_names["CONTAINER_MASS_TOPIC"], self.container_mass_callback, 1)

        # self.rover_node.node.create_subscription(FourInOne,
        #                                             self.rover_node.el_names["FOUR_IN_ONE_TOPIC"], self.four_in_one_callback, 1)

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
            "moisture": "0.0",
            "conductivity": "0.0",
            "ph": "0.0"
        }
        self.rover_node.rover_state_json['electronics']['sensors']['mass_sensors']["mass_drill"] = "0.0"
        self.rover_node.rover_state_json['electronics']['sensors']['mass_sensors']["mass_container"] = "0.0"


    def send_led_commands(self, subsystem, mode):

        match subsystem:
            case 'nav':
                self.send_mode("nav", mode)
            case 'hd':
                self.send_mode("hd", mode)
            case 'drill':
                self.send_mode("drill", mode)


    def send_mode(self, system, mode):
        pass
        # led = LEDMessage()
        # led.system = self.model.name_system[system]

        # match mode:
        #     case 'Manual':
        #         led.mode = 1
        #     case 'Manual Direct':
        #         led.mode = 2
        #     case 'Manual Inverse':
        #         led.mode = 3
        #     case 'Auto':
        #         led.mode = 4
        #     case 'Off':
        #         led.mode = 5
        #     case 'On':
        #         led.mode = 0
        #     case 'action':
        #         led.mode = 6

        # self.led_pub.publish(led)
    
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

    def drill_mass_callback(self, msg):
        self.rover_node.rover_state_json['electronics']['sensors']['mass_sensors']["mass_drill"] = msg.mass[1]
    
    def container_mass_callback(self, msg):
        self.rover_node.rover_state_json['electronics']['sensors']['mass_sensors']["mass_container"] = msg.mass[0]
    
    def four_in_one_callback(self, msg):
        self.rover_node.rover_state_json['electronics']['sensors']['four_in_one'] = {
            "temperature": msg.temperature,
            "moisture": msg.moisture,
            "conductivity": msg.conductivity,
            "ph": msg.ph
        }
    
    def bms_callback(self, msg):
        self.rover_node.rover_state_json['electronics']['power'] = {
            "voltage": msg.v_bat,
            "current": msg.current,
            "state": msg.status
        }


