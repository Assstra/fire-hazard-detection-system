import serial
import pyudev
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

baudrate = 115200

devices: Dict = {
    "002": {
        "coordinates": {
            "x": 0.0,
            "y": 0.0
        },
        "last_healthcheck": 0
    },
    "003": {
        "coordinates": {
            "x": 0.0,
            "y": 0.0
        },
        "last_healthcheck": 0
    },
    "004": {
        "coordinates": {
            "x": 0.0,
            "y": 0.0
        },
        "last_healthcheck": 0
    },
    "005": {
        "coordinates": {
            "x": 0.0,
            "y": 0.0
        },
        "last_healthcheck": 0
    }
}

fileAddress = r'logs.txt'
debugMode = True

class Device:
    port: str
    baudrate: int
    
    def __init__(self, port: str, baudrate: int):
        self.port = port
        self.baudrate = baudrate
        self.ser = serial.Serial(self.port, self.baudrate)
        print(f"Device created with serial instance: {self.ser}")
        
    def __str__(self) -> str:
        return f"Device with port {self.port} and baudrate {self.baudrate}"
           
class Message:
    type: str
    device_id: str
    timestamp: str
    additional_info: str
    
    def check_integrity(self, message: str):
        if not message or len(message) < 11:
            return False

        type = message[0] # 1st char

        # Available message types: Healthcheck or Alert
        if not (type in ['H', 'A']):
            return False

        return True
    
    def __init__(self, string: str) -> None:
        # e.g. H-999-2600-300
        # H => Healthcheck
        # 999 => DeviceID
        # 2600 => Timestamp
        # 300 => CO Sensor value
        
        # Clean message
        string = string.replace('\r', '').replace('\n', '')
        if "Receive Data(" in string:
            string = string.split("Receive Data(")[-1]
        string = string.rstrip('#')
        
        # Check message integrity
        if self.check_integrity(string):
            message_array = string.split("-", 4)
            self.type, self.device_id, self.timestamp, self.additional_info = message_array
        else:
            print("[ERROR] - Message is too short or message type is not recognized")

    def __str__(self) -> str:
        return f"Message({self.type}-{self.device_id}-{self.timestamp}-{self.additional_info})"

# Print debug messages
def debug_print(message: str):
    if debugMode:
        print("[DEBUG] - ", message)

def push_message_to_logfile(message: Message):
    
    log_message = time.strftime("%Y-%m-%d %H:%M:%S") + ": [ Alert ] -" + "Device: " + message.device_id + ", " + message.additional_info
    
    if os.path.isfile(fileAddress):
        with open(fileAddress, 'a') as data:
            data.write(log_message + '\n')
            debug_print(f"Data added to log file at location {fileAddress}")
    else:
        with open(fileAddress, 'w') as data:
            data.write(log_message + '\n')
            debug_print(f"File created & data added to log file at location {fileAddress}")

def get_device_tty():
    context = pyudev.Context()
    for device in context.list_devices(subsystem='tty'):
        if device.device_node and device.device_node.startswith('/dev/ttyUSB'):
            name = device.properties.get('ID_MODEL_FROM_DATABASE', '')
            vendor = device.properties.get('ID_VENDOR_FROM_DATABASE', '')
            if (name == "FT232 Serial (UART) IC" and
                vendor == "Future Technology Devices International, Ltd"):
                return str(device.device_node)
        return None

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
    for _ in range(2):  # Publish a few times for reliability
        pub.publish(pose)
        rate.sleep()

def read_until_marker(device, end_marker="#"):
    buffer = ""
    while True:
        char = device.ser.read().decode('utf-8', errors='ignore')
        buffer += char
        if char == end_marker:
            return buffer

def handle_message_by_type(message: Message):
    
    type = message.type
    time = datetime.now()
    
    print("[INFO] - Message:")
    print("Type: ", message.type)
    print("Device: ", message.device_id)
    
    if message.additional_info:
        print("Additional info: ", message.additional_info)
    
    push_message_to_logfile(message)
    
    # Healthcheck message
    if type == "H":
        
        debug_print(f"Healthcheck received from {message.device_id}: {message}")
        debug_print(f"Status of device {message.device_id} before: {devices[message.device_id]['last_healthcheck']}")
        
        devices[message.device_id]["last_healthcheck"] = time.time()
        debug_print(f"Status of device {message.device_id} after: {devices[message.device_id]['last_healthcheck']}")
        
    # Alert message
    elif type == "A":
        
        debug_print(f"Alert! received from {message.device_id}: {message}")
        
        coord = devices[message.device_id]["coordinates"]
        send_position_to_robot(coord["x"], coord["y"])
    else:
        print("[WARN] - Invalid message type")

def main():
    
    print("--- Program started: ready to receive messages ---")
    
    try:
        port = get_device_tty()
        debug_print(f"TTY port found: {port}")
        if port is None:
            print("[ERROR]: No valid TTY port found. Exiting.")
            return
    except Exception as e:
        print(f"[WARN]: Failed to parse message. Error: {e}")
        return
    
    device = Device(port, baudrate)
    debug_print(str(device))
    
    # Define timer to check for healthcheck message reception
    current_time = time.time()
    main.last_health_check = current_time

    while True:
        received_string = read_until_marker(device, end_marker="#")
        if received_string:
            debug_print(received_string)

            # Create message
            try:
                message = Message(received_string)
                debug_print(f"Message successfully created: {str(message)}")
            except Exception as e:
                print(f"[WARN]: Failed to parse message. Error: {e}")
                continue
        
            handle_message_by_type(message)
            
        # Check every device health
        current_time = time.time()

        if current_time - main.last_health_check >= 600:  # 10 minutes
            all_healthy = all(current_time - device_info["last_healthcheck"] <= 600 for device_info in devices.values())
            if all_healthy:
                print("[INFO] - All devices are healthy.")
            else:
                print("[WARNING] - Some devices are not healthy.")
                for device_id, device_info in devices.items():
                    if current_time - device_info["last_healthcheck"] >= 600:
                        print(f"[INFO] - Device {device_id} is not healthy. Coordinates: {device_info['coordinates']}")
            main.last_health_check = current_time

if __name__ == "__main__":
    main()
