from rclpy.node import Node
import requests
from requests.auth import HTTPBasicAuth

class NetworkMonitoring(Node):
    def __init__(self, rover_state):

        super().__init__('network_monitoring')
        
        self.rover_state = rover_state
        self.url_connected_device = 'http://169.254.55.1/rest/interface/wifi/registration-table'
        self.url_arp_table = 'http://169.254.55.1/rest/ip/arp'

        self.arp_list = None

        self.auth = HTTPBasicAuth('admin', 'XploreAntenna3')
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("Networking Node ready")
    
    def get_devices(self):
        self.arp_list = requests.get(self.url_arp_table, auth=self.auth, verify=False) # name, mac, ipv4, ipv6
        if self.arp_list.status_code == 200:
            data = self.arp_list.json()
            print(data)
        else:
            self.rover_state['rover']['network']['connected_devices'].append({"name": "No device"})


    def timer_callback(self):
        response = requests.get(self.url_connected_device, auth=self.auth, verify=False)
        cs = False
        if response.status_code == 200:
            data = response.json()
            print(data)
            if len(data) != 0:
                for idx, device in enumerate(data):
                    # LIST DEVICES
                    item = {
                        "name": device.get('ssid')
                    }
                    self.rover_state['rover']['network']['connected_devices'].append(item)

                    # SIGNAL STRENGTH
                    if(device.get('mac-address') == "D4:01:C3:DC:B9:77"):
                        self.rover_state['rover']['network']['signal_strength'] = device.get('signal')
                        cs = True
                
                if not cs:
                    self.rover_state['rover']['network']['signal_strength'] = '0.0'
            else:
                self.rover_state['rover']['network']['signal_strength'] = '1.0'
        else:
            self.rover_state['rover']['network']['signal_strength'] = '2.0'