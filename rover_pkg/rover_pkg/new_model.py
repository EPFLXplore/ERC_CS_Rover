from std_msgs.msg import String
from std_srvs.srv import SetBool
from custom_msg.srv import HDMode
import Drill, Navigation, HandlingDevice, Elec, json

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

        elif system == 1:
            # HD
            request = HDMode.Request()
            request.mode = mode

            print("HD mode:", mode)

            future = self.rover_node.hd_mode_service.call_async(request)
            future.add_done_callback(lambda f: service_callback(f))
        
            def service_callback(future):
                try:
                    response = future.result()
                    if response.system_mode == mode:
                        self.rover_node.rover_state_json['rover']['status']['systems']['handling_device']['status'] = 'Auto' if (mode == 3) else ('Manual Inverse' if (mode == 2) else ('Manual Direct' if (mode == 1) else 'Off'))
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
