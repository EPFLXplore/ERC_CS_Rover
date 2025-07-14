from std_msgs.msg import Float32
from std_srvs.srv import SetBool
from custom_msg.srv import DrillMode, ChangeModeSystem
from rover_pkg.drill_model import Drill
from rover_pkg.navigation_model import Navigation
from rover_pkg.handling_device_model import HandlingDevice
from rover_pkg.elec_model import Elec
from .states import SubSystems
    
'''
Authors: Ugo Balducci & Giovanni Ranieri
Year: 2023-2025
Description: General Model for all subsystems. Implements generic functions for cameras and changing mode for
subsystems.
'''
    
class NewModel:
    def __init__(self, rover_node):
        self.rover_node = rover_node

        # Create the different models
        self.Drill = Drill(rover_node)
        self.HD = HandlingDevice(rover_node)
        self.Nav = Navigation(rover_node)
        self.Elec = Elec(rover_node)

        # Bandwidth subscription for cameras
        self.rover_node.node.create_subscription(Float32, "/ROVER/bw_camera_cs_0", self.cs_data_rates_0, 10)
        self.rover_node.node.create_subscription(Float32, "/ROVER/bw_camera_cs_1", self.cs_data_rates_1, 10)
        self.rover_node.node.create_subscription(Float32, "/ROVER/bw_camera_cs_2", self.cs_data_rates_2, 10)
        self.rover_node.node.create_subscription(Float32, "/ROVER/bw_camera_cs_3", self.cs_data_rates_3, 10)
        
        self.rover_node.node.create_subscription(Float32, "/NAV/bw_camera_nav_0", self.nav_data_rates_0, 10)
        self.rover_node.node.create_subscription(Float32, "/NAV/bw_camera_nav_1", self.nav_data_rates_1, 10)
        self.rover_node.node.create_subscription(Float32, "/NAV/bw_camera_nav_2", self.nav_data_rates_2, 10)
        
        self.rover_node.node.create_subscription(Float32, "/HD/bw_camera_hd_0", self.hd_data_rates_0, 10)

    '''
    Function handling the change of mode of a subsystem from CS.
    The service is forwarded to the subsystem and the result is then sent back to the CS.
    If everything goes well, the leds are updated.
    '''
    async def change_mode_system_service(self, request, response):

        system = request.system
        mode = request.mode
        
        # --------------------------------------------------------------------
        # --------------------------------------------------------------------
        # NAVIGATION SYSTEM
        if system == 0:
            self.send_nav_service(system, mode)

            response.new_mode = 0
            response.error_type = 0
            response.error_message = "error_message"
            return response
        
        # --------------------------------------------------------------------
        # --------------------------------------------------------------------
        # HD SYSTEM
        elif system == 1:
            self.send_hd_service(system, mode)
        
            response.new_mode = 0
            response.error_type = 0
            response.error_message = "error_message"

            return response

        # --------------------------------------------------------------------
        # --------------------------------------------------------------------
        # DRILL SYSTEM
        elif system == 2:
            req = DrillMode.Request()
            req.mode = mode
            future = self.rover_node.drill_service.call_async(req)
            future.add_done_callback(lambda f: self.service_callback_drill(f, mode, response))
            
            response.new_mode = 0
            response.error_type = 0
            response.error_message = "error_message"
            return response    
    
    '''
    Utility function
    '''
    def send_nav_service(self, system, mode):
        req = ChangeModeSystem.Request()
        req.system = system
        req.mode = mode

        future = self.rover_node.nav_service.call_async(req)
        future.add_done_callback(lambda f: self.service_callback_nav(f, mode))
        
    '''
    Utility function
    '''
    def send_hd_service(self, system, mode):
        req = ChangeModeSystem.Request()
        req.system = system
        req.mode = mode

        future = self.rover_node.hd_mode_service.call_async(req)
        future.add_done_callback(lambda f: self.service_callback_hd(f, mode))
            
    '''
    Function callback for changing navigation mode
    '''
    def service_callback_nav(self, future, mode):
        try:
            response = future.result()
            if response.error_type == 0 and response.new_mode == mode:
                self.rover_node.rover_state_json['rover']['status']['systems']['navigation']['status'] = 'Auto' if (mode == 2) else ('Ackermann' if (mode == 1) else ('Omni' if (mode == 2) else 'Off'))
                #self.Elec.send_led_commands(SubSystems.NAVIGATION, mode)
                self.rover_node.switching_nav = False  # Update shared attribute instead of local variable

            else:
                self.rover_node.node.get_logger().info("Error in nav service response callback: " + response.error_message)
        except Exception as e:
            self.rover_node.node.get_logger().info("Error in nav service call: " + str(e))
    
    '''
    Function callback for changing HD mode
    '''
    def service_callback_hd(self, future, mode):
        try:
            response = future.result()
            if response.error_type == 0 and response.new_mode == mode:
                self.rover_node.rover_state_json['rover']['status']['systems']['handling_device']['status'] = 'Auto' if (mode == 3) else ('Manual Inverse' if (mode == 2) else ('Manual Direct' if (mode == 1) else 'Off'))
                #elf.Elec.send_led_commands(SubSystems.HANDLING_DEVICE, mode)
                self.rover_node.switching_hd = False
            else:
                self.rover_node.node.get_logger().info("Error in hd service response callback: " + response.error_message)
        except Exception as e:
            self.rover_node.node.get_logger().info("Error in hd service call: " + str(e))
    
    '''
    Function callback for changing drill mode
    '''   
    def service_callback_drill(self, future, mode, response):
        try:
            response_drill = future.result()
            if response_drill.error_type == 0 and response_drill.system_mode == mode:
                self.rover_node.rover_state_json['rover']['status']['systems']['drill']['status'] = 'On' if (mode == 1) else 'Off'
                #self.Elec.send_led_commands(SubSystems.DRILL, mode)
            else:
                self.rover_node.node.get_logger().info("Error in drill service response callback: " + response.error_message)

        except Exception as e:
            self.rover_node.node.get_logger().info("Error in drill service call: " + str(e))  

    '''
    Function callback for changing a camera mode
    '''
    def service_callback_camera(self, future, subsystem, index, activate):
        try:
            response_camera = future.result()
            if response_camera.success == True:
                self.rover_node.rover_state_json['cameras'][subsystem][index]['status'] = activate
            else:
                self.rover_node.node.get_logger().info("Error in camera service response callback: " + response_camera.error_message)

        except Exception as e:
            self.rover_node.node.get_logger().info("Error in camera service call: " + str(e)) 
    
    '''
    Function handling the change of mode of a camera
    The service is forwarded to the right camera node and the result is then sent back to the CS.
    '''
    def change_mode_camera_service(self, request, response):
        system = request.subsystem
        index = request.camera_name
        activate = request.activate
        
        # CS
        if(system == "rover"):
            req = SetBool.Request()
            req.data = True if activate else False

            match index:
                case "Left": # GOOD DO NOT TOUCH
                    future = self.rover_node.camera_cs_service_1.call_async(req)
                    future.add_done_callback(lambda f: self.service_callback_camera(f, system, index, activate))

                case "UpRight": # GOOD DO NOT TOUCH
                    future = self.rover_node.camera_cs_service_0.call_async(req)
                    future.add_done_callback(lambda f: self.service_callback_camera(f, system, index, activate))

                case "Right": # GOOD DO NOT TOUCH
                    future = self.rover_node.camera_cs_service_2.call_async(req)
                    future.add_done_callback(lambda f: self.service_callback_camera(f, system, index, activate))
                    
                case "UpLeft": # GOOD DO NOT TOUCH
                    future = self.rover_node.camera_cs_service_3.call_async(req)
                    future.add_done_callback(lambda f: self.service_callback_camera(f, system, index, activate))
                    
                case "Other1": # GOOD DO NOT TOUCH
                    future = self.rover_node.camera_cs_service_4.call_async(req)
                    future.add_done_callback(lambda f: self.service_callback_camera(f, system, index, activate))
                    
                case "Other2": # GOOD DO NOT TOUCH
                    future = self.rover_node.camera_cs_service_5.call_async(req)
                    future.add_done_callback(lambda f: self.service_callback_camera(f, system, index, activate))
            

            response.error_type = 0
            response.error_message = "error_message"
            return response
        
        # NAV
        if(system == "navigation"):
            req = SetBool.Request()
            req.data = True if activate else False

            match index:
                case "Front":
                    future = self.rover_node.camera_nav_service_0.call_async(req)
                    future.add_done_callback(lambda f: self.service_callback_camera(f, system, index, activate))
                
                case "Up1":
                    future = self.rover_node.camera_nav_service_1.call_async(req)
                    future.add_done_callback(lambda f: self.service_callback_camera(f, system, index, activate))

                case "Up2":
                    future = self.rover_node.camera_nav_service_2.call_async(req)
                    future.add_done_callback(lambda f: self.service_callback_camera(f, system, index, activate))
            
            
            response.error_type = 0
            response.error_message = "error_message"
            return response

        # HD
        if(system == "handling_device"):
            req = SetBool.Request()
            req.data = True if activate else False

            match index:
                case "Gripper":
                    future = self.rover_node.camera_hd_service_0.call_async(req)
                    future.add_done_callback(lambda f: self.service_callback_camera(f, system, index, activate))
            
            response.error_type = 0
            response.error_message = "error_message"
            return response
        

    '''
    Function handling the depth of the HD camera.
    '''
    def change_mode_camera_HD_service(self, request, response):
        
        req = SetBool.Request()
        req.data = True if request.data else False

        future = self.rover_node.change_camera_HD_mode_client.call_async(req)
        future.add_done_callback(lambda f: self.service_callback_camera_HD(f, request.data))
        
        response.success = True
        return response
    
    '''
    Function handling the depth of the NAV camera.
    '''
    def change_mode_camera_NAV_service(self, request, response):
        
        req = SetBool.Request()
        req.data = True if request.data else False

        future = self.rover_node.change_camera_NAV_mode_client.call_async(req)
        future.add_done_callback(lambda f: self.service_callback_camera_NAV(f, request.data))
        
        response.success = True
        return response
    
    def service_callback_camera_HD(self, future, activate):
        try:
            response_camera = future.result()
            if response_camera.success == True:
                self.rover_node.rover_state_json['cameras']['handling_device']['Gripper']['depth'] = activate
            else:
                self.rover_node.node.get_logger().info("Error in camera HD RGBD mode service response callback")

        except Exception as e:
            self.rover_node.node.get_logger().info("Error in camera HD RGBD mode service call: " + str(e)) 
    
    def service_callback_camera_NAV(self, future, activate):
        try:
            response_camera = future.result()
            if response_camera.success == True:
                self.rover_node.rover_state_json['cameras']['navigation']['Front']['depth'] = activate
            else:
                self.rover_node.node.get_logger().info("Error in camera NSV RGBD mode service response callback")

        except Exception as e:
            self.rover_node.node.get_logger().info("Error in camera NAV RGBD mode service call: " + str(e)) 
    
    
# ----------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------
# DATA RATES CAMERAS

    def cs_data_rates_0(self, msg):
        self.rover_node.rover_state_json['cameras']['rover']['UpRight']['data_rate'] = msg.data 

    def cs_data_rates_1(self, msg):
        self.rover_node.rover_state_json['cameras']['rover']['Left']['data_rate'] = msg.data 

    def cs_data_rates_2(self, msg):
        self.rover_node.rover_state_json['cameras']['rover']['Right']['data_rate'] = msg.data 
        
    def cs_data_rates_3(self, msg):
        self.rover_node.rover_state_json['cameras']['rover']['UpLeft']['data_rate'] = msg.data 
        
    def nav_data_rates_0(self, msg):
        self.rover_node.rover_state_json['cameras']['navigation']['Front']['data_rate'] = msg.data 

    def nav_data_rates_1(self, msg):
        self.rover_node.rover_state_json['cameras']['navigation']['Up1']['data_rate'] = msg.data 

    def nav_data_rates_2(self, msg):
        self.rover_node.rover_state_json['cameras']['navigation']['Up2']['data_rate'] = msg.data 

    def hd_data_rates_0(self, msg):        
        self.rover_node.rover_state_json['cameras']['handling_device']['Gripper']['data_rate'] = msg.data 


# ----------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------
# STATE CAMERAS

    def cs_states_0(self, msg):
        self.rover_node.rover_state_json['cameras']['rover']['UpRight']['status'] = msg.data 
        
        if not msg.data:
            self.rover_node.rover_state_json['cameras']['rover']['UpRight']['data_rate'] = "0.0"

    def cs_states_1(self, msg):
        self.rover_node.rover_state_json['cameras']['rover']['Left']['status'] = msg.data
        
        if not msg.data:
            self.rover_node.rover_state_json['cameras']['rover']['Left']['data_rate'] = "0.0"

    def cs_states_2(self, msg):
        self.rover_node.rover_state_json['cameras']['rover']['Right']['status'] = msg.data 
        
        if not msg.data:
            self.rover_node.rover_state_json['cameras']['rover']['Right']['data_rate'] = "0.0"
            
    def cs_states_3(self, msg):
        self.rover_node.rover_state_json['cameras']['rover']['UpLeft']['status'] = msg.data 
        
        if not msg.data:
            self.rover_node.rover_state_json['cameras']['rover']['UpLeft']['data_rate'] = "0.0"
            
    def cs_states_4(self, msg):
        self.rover_node.rover_state_json['cameras']['rover']['Other1']['status'] = msg.data 
        
        if not msg.data:
            self.rover_node.rover_state_json['cameras']['rover']['Other1']['data_rate'] = "0.0"
            
    def cs_states_5(self, msg):
        self.rover_node.rover_state_json['cameras']['rover']['Other2']['status'] = msg.data 
        
        if not msg.data:
            self.rover_node.rover_state_json['cameras']['rover']['Other2']['data_rate'] = "0.0"
        
    def nav_states_0(self, msg):
        self.rover_node.rover_state_json['cameras']['navigation']['Front']['status'] = msg.data 
        
        if not msg.data:
            self.rover_node.rover_state_json['cameras']['navigation']['Front']['data_rate'] = "0.0"

    def nav_states_1(self, msg):
        self.rover_node.rover_state_json['cameras']['navigation']['Up1']['status'] = msg.data 
        
        if not msg.data:
            self.rover_node.rover_state_json['cameras']['navigation']['Up1']['data_rate'] = "0.0"

    def nav_states_2(self, msg):
        self.rover_node.rover_state_json['cameras']['navigation']['Up2']['status'] = msg.data 
        
        if not msg.data:
            self.rover_node.rover_state_json['cameras']['navigation']['Up2']['data_rate'] = "0.0"
        
    def hd_states_0(self, msg):
        self.rover_node.rover_state_json['cameras']['handling_device']['Gripper']['status'] = msg.data 
        
        if not msg.data:
            self.rover_node.rover_state_json['cameras']['handling_device']['Gripper']['data_rate'] = "0.0"


