from std_msgs.msg import String
from std_srvs.srv import SetBool
from custom_msg.srv import HDMode, DrillMode, ChangeModeSystem
from rover_pkg.drill_model import Drill
from rover_pkg.navigation_model import Navigation
from rover_pkg.handling_device_model import HandlingDevice
from rover_pkg.elec_model import Elec
import json

class NewModel:
    def __init__(self, rover_node):
        self.rover_node = rover_node


        # FOR NOW WILL BE CHANGED
        self.systems_to_name = {
            0: "nav",
            1: "hd",
            2: "camera",
            3: "drill"
        }

        self.name_system = {
            "nav": 0,
            "hd": 1,
            "drill": 2
        }

        self.nav_to_name = {
            0: "Off",
            1: "Manual",
            2: "Auto"
        }

        self.hd_to_name = {
            0: "Off",
            1: "Manual Direct",
            2: "Auto",
            3: "Manual Inverse"
        }

        self.drill_to_name = {
            0: "Off",
            1: "On"
        }

        self.Drill = Drill(rover_node)
        self.HD = HandlingDevice(rover_node)
        self.Nav = Navigation(rover_node)
        self.Elec = Elec(rover_node, self)

    async def change_mode_system_service(self, request, response):

        system = request.system
        mode = request.mode
        print(f"system: {system}")
        print(f"mode: {mode}")
        '''
        # test leds
        if system == 0:
            self.Elec.send_led_commands(self.systems_to_name[system], self.nav_to_name[mode])
        
        elif system == 1:
            self.Elec.send_led_commands(self.systems_to_name[system], self.hd_to_name[mode])
        
        elif system == 3:
            self.Elec.send_led_commands(self.systems_to_name[system], self.drill_to_name[mode])
        
        return response
        '''
        
        # --------------------------------------------------------------------
        # --------------------------------------------------------------------
        # NAVIGATION SYSTEM
        if system == 0:
            # ADD LEDS WHEN SERVICE IS DONE ON NAV
            
            req = ChangeModeSystem.Request()
            req.system = system
            req.mode = mode

            future = self.rover_node.nav_service.call_async(req)
            future.add_done_callback(lambda f: self.service_callback_nav(f, mode))
            
            
            response.systems_state = ""
            response.error_type = 0
            response.error_message = "error_message"
            return response
# ----------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------
        # --------------------------------------------------------------------
        # --------------------------------------------------------------------
        # HD SYSTEM
        elif system == 1:
            req = HDMode.Request()
            req.mode = mode

            future = self.rover_node.hd_mode_service.call_async(req)
            future.add_done_callback(lambda f: self.service_callback_hd(f, mode))
        
            
            return response

# ----------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------
        # --------------------------------------------------------------------
        # --------------------------------------------------------------------
        # DRILL SYSTEM
        elif system == 3:
            req = DrillMode.Request()
            req.mode = mode
            future = self.rover_node.drill_service.call_async(req)
            future.add_done_callback(lambda f: self.service_callback_drill(f, mode, response))
            
            return response    
    

    def service_callback_nav(self, future, mode):
        try:
            response = future.result()
            
            if response.error_type == 0 and response.system_mode == mode:
                self.rover_node.rover_state_json['rover']['status']['systems']['navigation']['status'] = 'Auto' if (mode == 2) else ('Manual' if (mode == 1) else 'Off')
                #self.Elec.send_led_commands(self.systems_to_name[system], self.hd_to_name[mode])
            else:
                log_error(self.rover_node, "Error in nav service response callback: " + response.error_message)
        except Exception as e:
            log_error(self.rover_node, "Error in nav service call: " + str(e))
       
    def service_callback_hd(self, future, mode):
        try:
            response = future.result()
            
            if response.error_type == 0 and response.system_mode == mode:
                self.rover_node.rover_state_json['rover']['status']['systems']['handling_device']['status'] = 'Auto' if (mode == 3) else ('Manual Inverse' if (mode == 2) else ('Manual Direct' if (mode == 1) else 'Off'))
                #self.Elec.send_led_commands(self.systems_to_name[system], self.hd_to_name[mode])
            else:
                log_error(self.rover_node, "Error in hd service response callback: " + response.error_message)
        except Exception as e:
            log_error(self.rover_node, "Error in hd service call: " + str(e))
            
    def service_callback_drill(self, future, mode, response):
        try:
            response_drill = future.result()
            if response_drill.error_type == 0 and response_drill.system_mode == mode:
                self.rover_node.rover_state_json['rover']['status']['systems']['drill']['status'] = 'On' if (mode == 1) else 'Off'
                self.Elec.send_led_commands("drill", self.drill_to_name[mode])
            else:
                log_error(self.rover_node, "Error in drill service response callback: " + response.error_message)

        except Exception as e:
            log_error(self.rover_node, "Error in drill service call: " + str(e))  

    def service_callback_camera(self, future, subsystem, index, activate):
        try:
            response_camera = future.result()
            if response_camera.error_type == 0:
                self.rover_node.rover_state_json['rover']['cameras'][subsystem][index]['status'] = True if (activate == 1) else False
            else:
                log_error(self.rover_node, "Error in camera service response callback: " + response_camera.error_message)

        except Exception as e:
            log_error(self.rover_node, "Error in camera service call: " + str(e)) 
    

    def change_mode_camera_service(self, request, response):
        system = request.subsystem
        index = request.camera_name
        activate = request.activate
        
        # CS
        if(system == 2):
            # we have 4 cameras
            req = SetBool.Request()
            req.data = True if activate else False

            match index:
                case "Front":
                    future = self.rover_node.camera_cs_service_0.call_async(req)
                    future.add_done_callback(lambda f: self.service_callback_camera(f, system, index, activate))
                
                case "Left":
                    future = self.rover_node.camera_cs_service_1.call_async(req)
                    future.add_done_callback(lambda f: self.service_callback_camera(f, system, index, activate))

                case "Right":
                    future = self.rover_node.camera_cs_service_2.call_async(req)
                    future.add_done_callback(lambda f: self.service_callback_camera(f, system, index, activate))

                case "Behind":
                    future = self.rover_node.camera_cs_service_3.call_async(req)
                    future.add_done_callback(lambda f: self.service_callback_camera(f, system, index, activate))
            
            
            response.error_type = 0
            response.error_message = "error_message"
            return response
        
        # NAV
        if(system == 0):
            # we have 4 cameras
            req = SetBool.Request()
            req.data = True if activate else False

            match index:
                case "Up1":
                    future = self.rover_node.camera_nav_service_0.call_async(req)
                    future.add_done_callback(lambda f: self.service_callback_camera(f, system, index, activate))
                
                case "Up2":
                    future = self.rover_node.camera_nav_service_1.call_async(req)
                    future.add_done_callback(lambda f: self.service_callback_camera(f, system, index, activate))

                case "Front":
                    future = self.rover_node.camera_nav_service_2.call_async(req)
                    future.add_done_callback(lambda f: self.service_callback_camera(f, system, index, activate))

                case "Around360":
                    future = self.rover_node.camera_nav_service_3.call_async(req)
                    future.add_done_callback(lambda f: self.service_callback_camera(f, system, index, activate))
            
            
            response.error_type = 0
            response.error_message = "error_message"
            return response

        # HD
        if(system == 1):
            # we have 2 cameras
            req = SetBool.Request()
            req.data = True if activate else False

            match index:
                case "Gripper":
                    future = self.rover_node.camera_hd_service_0.call_async(req)
                    future.add_done_callback(lambda f: self.service_callback_camera(f, system, index, activate))
                
                case "Other":
                    future = self.rover_node.camera_hd_service_1.call_async(req)
                    future.add_done_callback(lambda f: self.service_callback_camera(f, system, index, activate))
            
            response.error_type = 0
            response.error_message = "error_message"
            return response
        
        # SC
        if(system == 3):
            # we have 2 cameras
            req = SetBool.Request()
            req.data = True if activate else False

            match index:
                case "Main":
                    future = self.rover_node.camera_sc_service_0.call_async(req)
                    future.add_done_callback(lambda f: self.service_callback_camera(f, system, index, activate))
            
            response.error_type = 0
            response.error_message = "error_message"
            return response

# ----------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------
    
def log_error(node, error_message):
    node.rover_state_json['rover']['status']['errors'] = node.rover_state_json['rover']['status']['errors'].append(error_message)

def log_warning(node, warning_message):
    node.rover_state_json['rover']['status']['warnings'] = node.rover_state_json['rover']['status']['warnings'].append(warning_message)

'''
'''
def response_service(node, response, error_type, error_message):
        res_sub_systems = {}
        sub_systems_status = node.rover_state_json['rover']['status']['systems']
        res_sub_systems['navigation'] = sub_systems_status['navigation']['status']
        res_sub_systems['handling_device'] = sub_systems_status['handling_device']['status']
        res_sub_systems['drill'] = sub_systems_status['drill']['status']
        res_sub_systems['cameras'] = sub_systems_status['cameras']['status']

        response.systems_state = json.dumps(res_sub_systems)
        response.error_type = error_type
        response.error_message = error_message

        return response
'''
'''
