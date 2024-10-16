from custom_msg.msg import LedsCommand, Led, MassArray, FourInOne, Voltage, Imu, Mag
class Elec:
    def __init__(self, rover_node, model):
        self.rover_node = rover_node
        self.model = model

        # 0 -> nav, 1 -> hd, 2 -> drill

        self.leds = self.rover_node.node.create_publisher(LedsCommand, 
                                                             self.rover_node.el_names["LED_COM_TOPIC"], 1)

        self.rover_node.node.create_subscription(MassArray, 
                                                  self.rover_node.el_names["DRILL_MASS_TOPIC"], self.drill_mass_callback, 1)

        self.rover_node.node.create_subscription(MassArray,
                                                    self.rover_node.el_names["CONTAINER_MASS_TOPIC"], self.container_mass_callback, 1)

        self.rover_node.node.create_subscription(FourInOne,
                                                    self.rover_node.el_names["FOUR_IN_ONE_TOPIC"], self.four_in_one_callback, 1)

        self.rover_node.node.create_subscription(Voltage, self.rover_node.el_names["VOLTAGE_TOPIC"], self.voltage_callback, 1)

        self.rover_node.node.create_subscription(Imu, self.rover_node.el_names['IMU_TOPIC'], self.imu_callback, 1)

        self.rover_node.node.create_subscription(Mag, self.rover_node.el_names['MAG_RAW_TOPIC'], self.mag_callback, 1)

        self.rover_node.node.create_subscription(Mag, self.rover_node.el_names['POTENTIOMETER_TOPIC'], self.potentiometer_callback, 1)


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
                led.mode = 1
            case 'Manual Direct':
                led.mode = 2
            case 'Manual Inverse':
                led.mode = 3
            case 'Auto':
                led.mode = 4
            case 'Off':
                led.mode = 5
            case 'On':
                led.mode = 0
            case 'action':
                led.mode = 6

        command = LedsCommand()
        command.leds = [led]
        self.leds.publish(command)

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
    
    def voltage_callback(self, msg):
        self.rover_node.rover_state_json['electronics']['sensors']['voltmeter'] = {
            "voltage": msg.voltage
        }
    
    def imu_callback(self, msg):
        self.rover_node.rover_state_json['electronics']['sensors']['imu'] = {
            "xyzw": msg.orientation,
            "orientation_covariance": msg.imu.orientation_cov,
            "ang_vel_cov": msg.imu.angular_velocity_covariance,
            "linear_accel_cov": msg.imu.linear_acceleration_covariance,
            "ang_vel": msg.imu.angular_velocity,
            "linear_accel": msg.imu.linear_acceleration
        }
    
    def mag_callback(self, msg):
        self.rover_node.rover_state_json['electronics']['sensors']['magnetic'] = {
            "mag_field": msg.mag_raw.magnetic_field,
            "mag_field_cov": msg.mag_raw.magnetic_field_covariance
        }
    
    def potentiometer_callback(self, msg):
        self.rover_node.rover_state_json['electronics']['sensors']['potentiometer'] = {
            "position": 
        }
    

