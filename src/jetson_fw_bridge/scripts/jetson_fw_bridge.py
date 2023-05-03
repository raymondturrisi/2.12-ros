#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import serial

class ArduinoSerial(object):
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.serial = None

    def connect(self):
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
            rospy.loginfo("Connected to Arduino on %s at %d baudrate", self.port, self.baudrate)
        except Exception as e:
            rospy.logerr("Failed to connect to Arduino: %s", str(e))

    def disconnect(self):
        if self.serial is not None:
            self.serial.close()

    def send_message(self, message):
        if self.serial is not None:
            self.serial.write((message + "\r\n").encode('utf-8'))

    def read_messages(self):
        if self.serial is not None:
            while not rospy.is_shutdown():
                try:
                    message = self.serial.readline().decode("utf-8").strip()
                    if message:
                        rospy.loginfo("Received message from Arduino: %s", message)
                except Exception as e:
                    rospy.logerr("Failed to read from Arduino: %s", str(e))

class ROSArduinoBridge(object):
    def __init__(self):
        self.serial = None
        self.ctrl_subscriber = None
        self.lift_subscriber = None

    def run(self):
        rospy.init_node('jetson_fw_bridge')
        self.connect_to_arduino()
        self.ctrl_subscriber = rospy.Subscriber("/to_fw/ctrl", String, self.ctrl_callback)
        self.lift_subscriber = rospy.Subscriber("/to_fw/lift", String, self.lift_callback)
        self.serial.read_messages()

    def connect_to_arduino(self):
        port = rospy.get_param("~serial_port", "/dev/ttyACM0")
        baudrate = rospy.get_param("~baudrate", 9600)
        self.serial = ArduinoSerial(port, baudrate)
        self.serial.connect()

    def disconnect_from_arduino(self):
        if self.serial is not None:
            self.serial.disconnect()

    def ctrl_callback(self, msg):
        # Relay control message to Arduino
        self.serial.send_message(msg.data)

    def lift_callback(self, msg):
        # Relay lift message to Arduino
        self.serial.send_message(msg.data)

if __name__ == "__main__":
    bridge = ROSArduinoBridge()
    try:
        bridge.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        bridge.disconnect_from_arduino()
