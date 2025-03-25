from rclpy.node import Node
import rclpy
import requests
from requests.auth import HTTPBasicAuth
import json
from ping3 import ping
from std_msgs.msg import String

class NetworkMonitoring(Node):
    def __init__(self, rover_state, node):

        super().__init__('network_monitoring')
        
        self.node = node
        self.rover_state = rover_state
        self.subnet = '169.254.55'
        self.url_connected_device = f"http://{self.subnet}.1/rest/interface/wifi/registration-table"
        self.auth = HTTPBasicAuth('admin', 'XploreAntenna3')
        self.devices_connected = []
        self.wireless_connection = None
        self.logs = None
        
        with open("/home/xplore/dev_ws/src/rover_pkg/rover_pkg/ip_names.json") as json_file:
            self.ip_names = dict(json.load(json_file))

        self.wireless_devices_timer = self.create_timer(2.0, self.retrieve_network_info)
        self.get_logger().info("Networking Node ready")
        
    def get_wireless_devices(self):
        response = requests.get(self.url_connected_device, auth=self.auth, verify=False)        
        if response.status_code == 200:
            self.wireless_connection = response.json()
            
            for idx, device in enumerate(self.wireless_connection):
                
                # Signal strength of Antenna mast
                if(device.get('mac-address') == "D4:01:C3:DC:B9:77"):
                    self.rover_state['rover']['network']['signal_strength'] = device.get('signal')
                    break

    
    def get_logs(self):
        response = requests.get(f"http://{self.subnet}.1/rest/log",
                                    auth=self.auth, verify=False) 

        if response.status_code == 200:
            self.logs = response.json()
    
    def retrieve_network_info(self):
        self.get_wireless_devices()
        self.get_logs()