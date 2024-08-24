from std_msgs.msg import String
from std_srvs.srv import SetBool
from custom_msg.srv import HDMode, DrillMode
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

    def update_metrics(self, metrics):
        self.rover_node.rover_state_json['rover']['hardware'] = json.loads(metrics.data)

    def change_mode_system_service(self, request, response):

        system = request.system
        mode = request.mode

        # --------------------------------------------------------------------
        # --------------------------------------------------------------------
        # NAVIGATION SYSTEM
        if system == 0:
            mode_cmd = String()            
            if mode == 1:
                mode_cmd.data = "manual"
            elif mode == 2 or mode == 0:
                mode_cmd.data = "auto"

            # ADD LEDS WHEN SERVICE IS DONE ON NAV

            self.rover_node.nav_mode_pub.publish(mode_cmd)
            self.rover_node.rover_state_json['rover']['status']['systems']['navigation']['status'] = 'Auto' if (mode == 2) else ('Manual' if (mode == 1) else 'Off')
            return response_service(self.rover_node, response, 0, "No errors")

        # --------------------------------------------------------------------
        # --------------------------------------------------------------------
        # HD SYSTEM
        elif system == 1:
            request = HDMode.Request()
            request.mode = mode

            future = self.rover_node.hd_mode_service.call_async(request)
            future.add_done_callback(lambda f: service_callback(f))
        
            def service_callback(future):
                try:
                    response = future.result()
                    
                    if response.error_type == 0 and response.system_mode == mode:
                        self.rover_node.rover_state_json['rover']['status']['systems']['handling_device']['status'] = 'Auto' if (mode == 3) else ('Manual Inverse' if (mode == 2) else ('Manual Direct' if (mode == 1) else 'Off'))
                        self.Elec.send_led_commands(self.systems_to_name[system], self.hd_to_name[mode])
                        return response_service(self.rover_node, response, 0, "No errors")
                    else:
                        log_error(self.rover_node, 1, "Error in hd service response callback: " + response.error_message)
                        return response_service(self.rover_node, response, 1, "Error in hd service response callback: " + response.error_message)
                except Exception as e:
                    log_error(self.rover_node, 1, "Error in hd service call: " + str(e))
                    return response_service(self.rover_node, response, 1,  "Error in hd service call: " + str(e))
            
        # --------------------------------------------------------------------
        # --------------------------------------------------------------------
        # CAMERA SYSTEM
        elif system == 2:
            request = SetBool.Request()
            request.data = True if (mode == 1) else False

            future = self.rover_node.camera_service.call_async(request)
            future.add_done_callback(lambda f: service_callback(f))
        
            def service_callback(future):
                try:
                    response = future.result()
                    if response.success:
                        self.rover_node.rover_state_json['rover']['status']['systems']['cameras']['status'] = 'Stream' if (mode == 1) else 'Off'
                        return response_service(self.rover_node, response, 0, "No errors")
                    else:
                        #self.rover_node.rover_state_json['rover']['status']['systems']['cameras']['status'] = 'Off' if (mode == 1) else 'Stream'
                        log_error(self.rover_node, 1, "Error in camera service response callback: " + response.error_message)
                        return response_service(self.rover_node, response, 1, "Error in camera service response callback: " + response.error_message)

                except Exception as e:
                    log_error(self.rover_node, 1, "Error in camera service call: " + str(e))
                    return response_service(self.rover_node, response, 1,  "Error in camera service call: " + str(e))

        # --------------------------------------------------------------------
        # --------------------------------------------------------------------
        # DRILL SYSTEM
        elif system == 3:
            request = DrillMode.Request()
            request.mode = mode

            future = self.rover_node.drill_service.call_async(request)
            future.add_done_callback(lambda f: service_callback(f))
        
            def service_callback(future):
                try:
                    response = future.result()
                    if response.error_type == 0 and response.system_mode == mode:
                        self.rover_node.rover_state_json['rover']['status']['systems']['drill']['status'] = 'On' if (mode == 1) else 'Off'
                        self.Elec.send_led_commands(self.systems_to_name[system], self.drill_to_name[mode])
                        return response_service(self.rover_node, response, 0, "No errors")
                    else:
                        log_error(self.rover_node, 1, "Error in drill service response callback: " + response.error_message)
                        return response_service(self.rover_node, response, 1, "Error in drill service response callback: " + response.error_message)

                except Exception as e:
                    log_error(self.rover_node, 1, "Error in drill service call: " + str(e))
                    return response_service(self.rover_node, response, 1,  "Error in drill service call: " + str(e))

    
def log_error(node, error_type, error_message):
    error = { "type": error_type, "message": error_message }
    node.rover_state_json['rover']['status']['error'] = node.rover_state_json['rover']['status']['error'].append(error)

def log_warning(node, warning_type, warning_message):
    warning = { "type": warning_type, "message": warning_message }
    node.rover_state_json['rover']['status']['warning'] = node.rover_state_json['rover']['status']['warning'].append(warning)

def response_service(node, error_type, error_message, response):
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
