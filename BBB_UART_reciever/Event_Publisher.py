import rclpy
import time
import os
import serial
import time
import sys

from custom_msg.msg import Event
from rclpy.node import Node
from geormetry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header

class SubmarineEventPublisher(Node):

    def __init__(self):
        super().__init("Submarine_Event_Publisher")
        self.publisher_ = self.create_publisher(Event, 'events', 10)
        while (1):
            val = self.uart_receiver(port="/dev/ttyS1", baudrate=115200)

            msg = Event()

            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'ship 0' # set as appropriate
            msg.header = header

            pose = Pose()

            if (val["type"] == "position"): 
                pose.position = Point(val["x"], val["y"], val["z"])
                msg.event_type = "position" 
                msg.message = "position data"
                
            if (val["type"] == "orientation"): 
                pose.orientation = Quaternion(val["x"], val["y"], val["z"], w=0.0)
                msg.event_type = "orientation" 
                msg.message = "orientation data"

            if (val["type"] == "event"):
                msg.event_type = "event"
                msg.message = f"ship {val["sub_id"]} {val["status"]}"

            msg.pose = pose

            self.publisher_.publish(msg)
            self.get_logger().info(f'Published Event: type={msg.event_type} msg="{msg.message}"')


    def parse_frame(self, frame: str):

        frame = frame.strip()
        if not frame:
            return None

        frame_type = frame[0]

        # --- Position Frame ---
        if frame_type == 'P':
            try:
                _, x, y, z = frame.split(',')
                return {"type": "position", "x": int(x), "y": int(y), "z": int(z)}
            except ValueError:
                return None

        # --- Orientation Frame ---
        elif frame_type == 'O':
            try:
                _, x, y, z = frame.split(',')
                return {"type": "orientation", "x": float(x), "y": float(y), "z": float(z)}
            except ValueError:
                return None

        # --- Event Frame ---
        elif frame_type == 'E':
            try:
                sub_id = int(frame[1])
                status_char = frame[2].upper()
                if status_char not in ['F', 'L']:
                    raise ValueError
                status = "found" if status_char == 'F' else "lost"
                return {"type": "event", "sub_id": sub_id, "status": status}
            except (IndexError, ValueError):
                return None

        else:
            return None


    def uart_receiver(self, port="/dev/ttyS4", baudrate=115200):
        """
        Continuously read and parse UART4 data.
        Automatically checks that UART4 is enabled and accessible.
        """
        # --- Check that the UART4 device exists ---
        if not os.path.exists(port):
            print(f"[ERROR] UART device '{port}' not found!")
            print("--------------------------------------------------")
            print("👉 To enable UART4 on BeagleBone Black:")
            print("   1. Edit /boot/uEnv.txt and ensure these lines exist:")
            print("        enable_uboot_overlays=1")
            print("        uboot_overlay_addr4=/lib/firmware/BB-UART4-00A0.dtbo")
            print("   2. Save and reboot:")
            print("        sudo reboot")
            print("--------------------------------------------------")
            sys.exit(1)

        try:
            ser = serial.Serial(port, baudrate, timeout=1)
        except serial.SerialException as e:
            print(f"[ERROR] Could not open {port}: {e}")
            sys.exit(1)

        buffer = ""
        try:
            while True:
                data = ser.read().decode(errors='ignore')
                if data:
                    if data == '\n':
                        frame = buffer.strip()
                        buffer = ""
                        parsed = self.parse_frame(frame)
                        if parsed:
                            return parsed
                    else:
                        buffer += data
        except KeyboardInterrupt:
            return "interrupted"
        finally:
            ser.close()
