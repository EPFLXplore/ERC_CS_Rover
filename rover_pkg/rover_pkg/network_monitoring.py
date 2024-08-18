from rclpy.node import Node
import requests
from requests.auth import HTTPBasicAuth
from librouteros import connect
from librouteros.login import plain
import json
from ping3 import ping

class NetworkMonitoring(Node):
    def __init__(self, rover_state):

        super().__init__('network_monitoring')
        
        self.rover_state = rover_state
        self.subnet = '169.254.55'
        self.url_connected_device = 'http://169.254.55.1/rest/interface/wifi/registration-table'
        self.url_dhcp_server = 'http://169.254.55.1/rest/ip/'
        self.auth = HTTPBasicAuth('admin', 'XploreAntenna3')
        self.list_ips = []
        
        with open("/home/xplore/dev_ws/src/rover_pkg/rover_pkg/ip_names.json") as json_file:
            self.ip_names = dict(json.load(json_file))

        self.connectivity_cs = self.create_timer(1.0, self.timer_callback)
        self.ping_timer = self.create_timer(10.0, self.check_devices_in_subnet)
        self.get_logger().info("Networking Node ready")
    

    def ping_device(self, ip):
        responses = [ping(ip, timeout=1) for _ in range(3)]
        success_responses = [r for r in responses if r is not None]
        return len(success_responses) >= 2

    def check_devices_in_subnet(self, subnet):
        for i in range(2, 255):
            ip = f"{subnet}.{i}"
            if self.ping_device(ip):
                if ip not in self.list_ips:
                    self.list_ips.append(ip)     
                    name = self.ip_names[ip]
                    if name is not None:   
                        item = {
                            "name": name,
                            "ip": ip
                        }
                    else:
                        item = {
                            "ip": ip
                        }
                    
                    self.rover_state['rover']['network']['connected_devices'].append(item)
            
            else:
                if ip in self.list_ips:
                    # remove the device with this ip in the list
                    self.rover_state['rover']['network']['connected_devices'][:] = 
                    [device for device in self.rover_state['rover']['network']['connected_devices'] if 
                     device.get("ip") == ip] 
                    self.list_ips.remove(ip)


    def timer_callback(self):
        response = requests.get(self.url_connected_device, auth=self.auth, verify=False)
        cs = False
        if response.status_code == 200:
            data = response.json()
            if len(data) != 0:
                for idx, device in enumerate(data):
                    if(device.get('mac-address') == "D4:01:C3:DC:B9:77"):
                        self.rover_state['rover']['network']['signal_strength'] = device.get('signal')
                        cs = True
                        break
                
                if not cs:
                    self.rover_state['rover']['network']['signal_strength'] = '0.0'
            else:
                self.rover_state['rover']['network']['signal_strength'] = '1.0'
        else:
            self.rover_state['rover']['network']['signal_strength'] = '2.0'
        