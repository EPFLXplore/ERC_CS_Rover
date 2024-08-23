from std_msgs.msg import String
from std_srvs.srv import SetBool
from custom_msg.srv import HDMode
from rover_pkg.drill_model import Drill
from rover_pkg.navigation_model import Navigation
from rover_pkg.handling_device_model import HandlingDevice
from rover_pkg.elec_model import Elec
import json

systems_to_name = {
    0: "nav",
    1: "hd",
    2: "camera",
    3: "drill"
}

nav_to_name = {
    0: "Off",
    1: "Manual",
    2: "Auto"
}

hd_to_name = {
    0: "Off",
    1: "Manual Direct",
    2: "Auto",
    3: "Manual Inverse"
}

drill_to_name = {
    0: "Off",
    1: "On"
}

class NewModel:
    def __init__(self, rover_node):
        self.rover_node = rover_node

        self.Drill = Drill(rover_node)
        self.HD = HandlingDevice(rover_node)
        self.Nav = Navigation(rover_node)
        self.Elec = Elec(rover_node)

    def update_metrics(self, metrics):
        self.rover_node.rover_state_json['rover']['hardware'] = json.loads(metrics.data)

    def change_mode_system_service(self, request, response):

        system = request.system
        mode = request.mode
        print("Change:", system, mode)

        if system == 0:
            # NAV  
            if mode == 1:
                mode_cmd = String()
                mode_cmd.data = "manual"
                self.rover_node.nav_mode_pub.publish(mode_cmd)
            elif mode == 2 or mode == 0:
                mode_cmd = String()
                mode_cmd.data = "auto"
                self.rover_node.nav_mode_pub.publish(mode_cmd)
            
            self.rover_node.rover_state_json['rover']['status']['systems']['navigation']['status'] = 'Auto' if (mode == 2) else ('Manual' if (mode == 1) else 'Off')
            self.Elec.send_led_commands(systems_to_name[system], nav_to_name[mode])
        elif system == 1:
            # HD
            request = HDMode.Request()
            request.mode = mode

            #future = self.rover_node.hd_mode_service.call_async(request)
            #future.add_done_callback(lambda f: service_callback(f))
            self.rover_node.rover_state_json['rover']['status']['systems']['handling_device']['status'] = 'Auto' if (mode == 3) else ('Manual Inverse' if (mode == 2) else ('Manual Direct' if (mode == 1) else 'Off'))

        
            def service_callback(future):
                try:
                    response = future.result()
                    
                    if response.system_mode == mode:
                        print("OKE CHANGEEEE")
                        self.rover_node.rover_state_json['rover']['status']['systems']['handling_device']['status'] = 'Auto' if (mode == 3) else ('Manual Inverse' if (mode == 2) else ('Manual Direct' if (mode == 1) else 'Off'))
                        self.Elec.send_led_commands(systems_to_name[system], hd_to_name[mode])
                    else:
                        log_error(self.rover_node, 1, "Error in camera service callback")
                except Exception as e:
                    log_error(self.rover_node, 1, "Error in camera service callback: " + str(e))
            

        elif system == 2:
            # Camera
            # call boolset service to start/stop cameras
            request = SetBool.Request()
            request.data = True if (mode == 1) else False
            future = self.rover_node.camera_service.call_async(request)
            future.add_done_callback(lambda f: service_callback(f))
        
            def service_callback(future):
                try:
                    response = future.result()
                    if response.success:
                        self.rover_node.rover_state_json['rover']['status']['systems']['cameras']['status'] = 'Stream' if (mode == 1) else 'Off'
                    else:
                        self.rover_node.rover_state_json['rover']['status']['systems']['cameras']['status'] = 'Off' if (mode == 1) else 'Stream'
                        log_error(self.rover_node, 1, "Error in camera service callback")
                except Exception as e:
                    log_error(self.rover_node, 1, "Error in camera service callback: " + str(e))


        elif system == 3:
            # Drill
            self.rover_node.rover_state_json['rover']['status']['systems']['drill']['status'] = 'On' if (mode == 1) else 'Off'

            self.Elec.send_led_commands(systems_to_name[system], drill_to_name[mode])


        res_sub_systems = {}
        sub_systems_status = self.rover_node.rover_state_json['rover']['status']['systems']
        res_sub_systems['navigation'] = sub_systems_status['navigation']['status']
        res_sub_systems['handling_device'] = sub_systems_status['handling_device']['status']
        res_sub_systems['drill'] = sub_systems_status['drill']['status']
        res_sub_systems['cameras'] = sub_systems_status['cameras']['status']

        response.systems_state = json.dumps(res_sub_systems)
        response.error_type = 0
        response.error_message = "no errors"  
        return response

    
def log_error(node, error_type, error_message):
    error = { "type": error_type, "message": error_message }
    node.rover_state_json['rover']['status']['error'] = node.rover_state_json['rover']['status']['error'].append(error)

def log_warning(node, warning_type, warning_message):
    warning = { "type": warning_type, "message": warning_message }
    node.rover_state_json['rover']['status']['warning'] = node.rover_state_json['rover']['status']['warning'].append(warning)
