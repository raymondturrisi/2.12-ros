#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import serial
import sys
import threading

# Callback function to handle incoming messages
#TODO: Need to add multiple message types which is getting relayed TO the arduino
def nmea_callback(msg):
    global nmea_string
    nmea_string = msg.data
    rospy.loginfo("Received NMEA string: %s", nmea_string)

# Function to read messages from the Arduino
#TODO: Parse arduino messages and publish them to the correct topics
def read_from_arduino(arduino_serial):
    while not rospy.is_shutdown():
        try:
            msg = arduino_serial.readline().decode("utf-8").strip()
            if msg:
                rospy.loginfo("Received message from Arduino: %s", msg)
        except Exception as e:
            rospy.logerr("Failed to read from Arduino: %s", str(e))

# Establish a connection with the Arduino via USB
def connect_arduino(port, baudrate):
    try:
        arduino_connection = serial.Serial(port, baudrate, timeout=1)
        rospy.loginfo("Connected to Arduino on %s at %d baudrate", port, baudrate)
        return arduino_connection
    except Exception as e:
        rospy.logerr("Failed to connect to Arduino: %s", str(e))
        return None

if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node('jetson_fw_bridge')

    # Get ROS parameters for serial port and baudrate
    serial_port = rospy.get_param("~serial_port", "/dev/ttyACM0")
    baudrate = rospy.get_param("~baudrate", 9600)

    # Connect to the Arduino
    arduino_serial = connect_arduino(serial_port, baudrate)
    if arduino_serial is None:
        rospy.logerr("Failed to connect to Arduino, shutting down")
        sys.exit(1)

    # Subscribe to the NMEA string topic
    nmea_subscriber = rospy.Subscriber("nmea_topic", String, nmea_callback)

    # Set up a rate control for the main loop
    rate = rospy.Rate(10)  # 10 Hz

    nmea_string = None

    # Start a separate thread to read messages from the Arduino
    arduino_read_thread = threading.Thread(target=read_from_arduino, args=(arduino_serial,))
    arduino_read_thread.start()

    # Main loop
    while not rospy.is_shutdown():
        if nmea_string:
            # Send the NMEA string to the Arduino via the serial connection
            arduino_serial.write((nmea_string + "\r\n").encode('utf-8'))
            nmea_string = None

        # Sleep to maintain the desired rate
        rate.sleep()

    # Close the serial connection when shutting down
    arduino_serial.close()
