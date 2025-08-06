import serial
import time
import os
from datetime import datetime
from typing import Dict
import rospy
from geometry_msgs.msg import Pose

#   Device config
#   -------------------------------------
#   Node                        : EndDevice
#   Band Width                  : 125kHz
#   Spreading Factor            : 7
#   Effective Bitrate           : 5469bps
#   Channel                     : 3
#   PAN ID                      : 0502
#   Own Node ID                 : 0001
#   Destination ID              : 0002
#   Acknowledge                 : ON
#   Retry count                 : 3
#   Transfer Mode               : Payload
#   Receive Node ID information : OFF

port = "/dev/ttyUSB0"
baudrate = "115200"

devices: Dict = {
    "002": {
        "coordinates": {
            "x": 37.7749,
            "y": -122.4194
        },
        "last_healthcheck": 0
    },
    "003": {
        "coordinates": {
            "x": -20.7749,
            "y": 40.42738
        },
        "last_healthcheck": 0
    }
}

fileAddress = r'logs.txt'

class Device:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.ser = serial.Serial(self.port, self.baudrate)
        print(f"Device created with serial instance: {self.ser}")
        
class Message:
    def __init__(self, string: str) -> None:
        self.string = string
    
    def clean(self):
        self.string = self.string.replace('\r', '').replace('\n', '')
        if "Receive Data(" in self.string:
            self.string = self.string.split("Receive Data(")[-1]
        self.string = self.string.rstrip('#')
        
    def check_integrity(self):
        # e.g. H9992600300
        # H => Healthcheck
        # 999 => DeviceID
        # 2600300 => Timestamp
        
        # Check message integrity based on type
        self.clean()
        msg = self.string

        if not msg or len(msg) < 11:
            return False  # Minimum length for H message is 11

        type = msg[0] # 1st char

        # Available types: Healthcheck, Logs or Alert
        if not (type in ['H', 'L', 'A']):
            return False

        return True
        
    def split_elements(self):
        """
        Splits the message string into its elements: type, device_id, timestamp, and optionally sensor_value.
        Returns a dictionary with the extracted elements.
        """
        self.check_integrity()
        self.clean()
        msg = self.string

        return {
            "type": msg[0],
            "device_id": msg[1:4],
            "timestamp": msg[4:11],
            "additional_info": msg[11:]
        }

def send_position_to_robot(x, y, z=0.0, ox=0.0, oy=0.0, oz=0.0, ow=1.0):
    pub = rospy.Publisher('/alert', Pose, queue_size=10)
    rospy.init_node('alert_publisher', anonymous=True)
    rate = rospy.Rate(1)  # 1 Hz
    pose = Pose()
    
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = ox
    pose.orientation.y = oy
    pose.orientation.z = oz
    pose.orientation.w = ow
    
    rospy.loginfo(f"Publishing alert position: x={x}, y={y}, z={z}")
    for _ in range(5):  # Publish a few times for reliability
        pub.publish(pose)
        rate.sleep()

def read_until_marker(device, end_marker="#"):
    buffer = ""
    while True:
        char = device.ser.read().decode('utf-8', errors='ignore')
        buffer += char
        if char == end_marker:
            return buffer

def handle_message_type(message):
    
    type = message["type"]
    time = datetime.now()
    
    print("Type: ", message["type"])
    print("Device: ", message["device_id"])
    if message["additional_info"]:
        print("Additional info: ", message["additional_info"])

    # Healthcheck message
    if type == "H":
        devices[message["device_id"]] = time.time()
        
        log_message = time.strftime("%Y-%m-%d %H:%M:%S") + ": [" + message["type"] + "] " + "Device: " + message["device_id"] + ", " + message["additional_info"]
        if os.path.isfile(fileAddress):
            with open(fileAddress, 'a') as data:
                data.write(log_message)
        else:
            with open(fileAddress, 'w') as data:
                data.write(log_message)

    # Alert message    
    elif type == "A":
        coord = devices[message["device_id"]]["coordinates"]
        send_position_to_robot(coord["x"], coord["y"])
    else:
        print("Invalid message")

def main():
    device = Device(port, baudrate)
    main.last_health_check = time.time()

    while True:
        message = read_until_marker(device, end_marker="#")
        print("Full message:", message)
        
        # Parse message
        new_message = Message(message)
        decrypted_message = new_message.split_elements()
        
        handle_message_type(decrypted_message)
            
        # Check every device health
        current_time = time.time()

        if current_time - main.last_health_check >= 600:  # 10 minutes
            all_healthy = all(current_time - device_info["last_healthcheck"] <= 600 for device_info in devices.values())
            if all_healthy:
                print("All devices are healthy.")
            else:
                print("Some devices are not healthy.")
                for device_id, device_info in devices.items():
                    if current_time - device_info["last_healthcheck"] >= 600:
                        print(f"Device {device_id} is not healthy. Coordinates: {device_info['coordinates']}")
            main.last_health_check = current_time

if __name__ == "__main__":
    main()
