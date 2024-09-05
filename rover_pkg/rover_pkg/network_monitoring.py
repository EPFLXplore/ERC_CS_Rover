from rclpy.node import Node
import rclpy
import requests
from requests.auth import HTTPBasicAuth
import json
from ping3 import ping


class NetworkMonitoring(Node):
    def __init__(self, rover_state):

        super().__init__('network_monitoring')
        
        self.rover_state = rover_state
        self.subnet = '169.254.55'
        self.url_connected_device = f"http://{self.subnet}.1/rest/interface/wifi/registration-table"
        self.auth = HTTPBasicAuth('admin', 'XploreAntenna3')
        self.devices_connected = []
        self.wireless_connection = None
        
        with open("/home/xplore/dev_ws/src/rover_pkg/rover_pkg/ip_names.json") as json_file:
            self.ip_names = dict(json.load(json_file))

        #self.wireless_devices_timer = self.create_timer(2.0, self.get_wireless_devices)
        self.ping_cs_timer = self.create_timer(2.0, self.ping_cs)
        #self.check_static_devices_timer = self.create_timer(5.0, self.check_static_devices)
        
        self.get_logger().info("Networking Node ready")
        
    
    
    def ping_cs(self):
        ping("192.168.0.115", timeout=10)
        '''
        responses = [ping(f"{self.subn§et}.{2}", timeout=1) for _ in range(2)]
        cs = False
        success_responses = [r for r in responses if r is not None]
        if len(success_responses) >= 1 and self.wireless_connection is not None:
            for idx, device in enumerate(self.wireless_connection):
                if(device.get('mac-address') == "D4:01:C3:DC:B9:77"):
                    self.rover_state['rover']['network']['signal_strength'] = device.get('signal')
                    cs = True
                    break
            
            if not cs:
                self.rover_state['rover']['network']['signal_strength'] = '0.0' # mean that the cs is not in the wireless tab
        '''
    def ping_static_address(self, ip):
        responses = [ping(ip, timeout=1) for _ in range(2)]
        success_responses = [r for r in responses if r is not None]
        return len(success_responses) >= 1

    def check_static_devices(self):
        for ip, name in self.ip_names.items():
            if self.ping_static_address(ip):
                # device connected
                if ip not in self.devices_connected:
                    self.devices_connected.append(ip)     
                    name = self.ip_names[ip]
                    item = {
                            "name": name,
                            "ip": ip
                        }
                    
                    self.rover_state['rover']['network']['connected_devices'].append(item)
            
            else:
                if ip in self.devices_connected:
                    # remove the device with this ip in the list
                    self.rover_state['rover']['network']['connected_devices'][:] = [device for device in self.rover_state['rover']['network']['connected_devices'] if 
                     device.get("ip") != ip] 
                    self.devices_connected.remove(ip)

    def get_wireless_devices(self):
        response = requests.get(self.url_connected_device, auth=self.auth, verify=False)        
        if response.status_code == 200:
            self.wireless_connection = response.json()
            
        else:
            self.wireless_connection = None 