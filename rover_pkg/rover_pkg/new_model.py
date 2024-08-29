from std_msgs.msg import String
from std_srvs.srv import SetBool
from custom_msg.srv import HDMode, DrillMode
from rover_pkg.drill_model import Drill
from rover_pkg.navigation_model import Navigation
from rover_pkg.handling_device_model import HandlingDevice
from rover_pkg.elec_model import Elec
import json, rclpy, threading
import json, rclpy, threading

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

    def update_metrics(self, metrics):
        self.rover_node.rover_state_json['rover']['hardware'] = json.loads(metrics.data)

    async def change_mode_system_service(self, request, response):
    async def change_mode_system_service(self, request, response):

        system = request.system
        mode = request.mode
        print("fefef")

        # test leds
        '''
        print(f"system: {system}")
        print(f"mode: {mode}")

        if system == 0:
            self.Elec.send_led_commands(self.systems_to_name[system], self.nav_to_name[mode])
        
        elif system == 1:
            self.Elec.send_led_commands(self.systems_to_name[system], self.hd_to_name[mode])
        
        elif system == 2:
            self.Elec.send_led_commands(self.systems_to_name[system], self.drill_to_name[mode])
        
        return response
        '''
        
        # --------------------------------------------------------------------
        # --------------------------------------------------------------------
        # NAVIGATION SYSTEM
        if system == 0:
            # ADD LEDS WHEN SERVICE IS DONE ON NAV
            '''
            future = self.rover_node.nav_service.call_async(request)
            future.add_done_callback(lambda f: self.service_callback_nav(f, mode))
            '''
            
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
            request = HDMode.Request()
            request.mode = mode

            future = self.rover_node.hd_mode_service.call_async(request)
            future.add_done_callback(lambda f: self.service_callback_hd(f, mode))
        
            
            return response
        
# ----------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------
        # --------------------------------------------------------------------
        # --------------------------------------------------------------------
        # CAMERA SYSTEM
        elif system == 2:
            request = SetBool.Request()
            request.data = True if (mode == 1) else False

            future = self.rover_node.camera_service.call_async(request)
            future.add_done_callback(lambda f: self.service_callback_cam(f, mode))

            
            return response

# ----------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------
        # --------------------------------------------------------------------
        # --------------------------------------------------------------------
        # DRILL SYSTEM
        elif system == 3:
            request = DrillMode.Request()
            request.mode = mode

            request.send_parameter = True
            request.rotation_speed = 1.1
            request.distance_ratio = 2.2
            

            request.send_parameter = True
            request.rotation_speed = 1.1
            request.distance_ratio = 2.2
            
            future = self.rover_node.drill_service.call_async(request)
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
        
    def service_callback_cam(self, future, mode):
        try:
            response = future.result()
            if response.success:
                self.rover_node.rover_state_json['rover']['status']['systems']['cameras']['status'] = 'Stream' if (mode == 1) else 'Off'
            else:
                self.rover_node.rover_state_json['rover']['status']['systems']['cameras']['status'] = 'Off' if (mode == 1) else 'Stream'
                log_error(self.rover_node, "Error in camera service response callback: " + response.error_message)

        except Exception as e:
            log_error(self.rover_node, "Error in camera service call: " + str(e))
            
    def service_callback_drill(self, future, mode, response):
        try:
            response_drill = future.result()
            if response_drill.error_type == 0 and response_drill.system_mode == mode:
                self.rover_node.rover_state_json['rover']['status']['systems']['drill']['status'] = 'On' if (mode == 1) else 'Off'
                #self.Elec.send_led_commands(self.systems_to_name[system], self.drill_to_name[mode])
            else:
                log_error(self.rover_node, "Error in drill service response callback: " + response.error_message)

        except Exception as e:
            log_error(self.rover_node, "Error in drill service call: " + str(e))    
    

# ----------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------
        except Exception as e:
            log_error(self.rover_node, "Error in drill service call: " + str(e))    
    

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
