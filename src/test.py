#!/usr/bin/python

import rospy
from sensor_msgs.msg import NavSatFix
import serial
from std_msgs import *
import time

# Serial Port Reference
ser = None

# Publishers for Sensor Data
gps_pub = None
battery_pub = None


def process_gps(data):
    navSatFix = NavSatFix()
    args = data.split(',')
    results = {}
    for pair in args:
        name, value = pair.split('=')
        results[name] = value
    
    navSatFix.latitude = int(results['lat']) / 10000000.0
    navSatFix.longitude = int(results['long']) / 10000000.0
    navSatFix.altitude = int(results['alt']) / 1000.0

    gps_pub.publish(navSatFix)

topic_publisher_callback = {
    'gps' : process_gps
}
def process_message(message):
    args = message.split(";")
    topic = args[0]
    data = args[1]

    topic_publisher_callback[topic](data)

def run():
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        while ser.in_waiting > 0:
            message = ser.read_until('\n')
            process_message(message)
        
        rate.sleep()


def main():
    global gps_pub, ser
    rospy.init_node('embedded_controller_relay')

    while (ser is None):
        try:
            ser = serial.Serial('/dev/ttyACM0')
        except:
            rospy.loginfo('Unable to locate Teensy, please ensure communications.')
            time.sleep(3)
    rospy.loginfo('Connected to teensy.')

    # Initilize Publishers
    gps_pub = rospy.Publisher('gps', NavSatFix, queue_size=5)
    
    # Function to continullay relay data between the Jetson and Teensy
    run()

    # End of Run tasks
    rospy.loginfo('Disconnecting teensy.')
    ser.close()
    rospy.loginfo('Shutting down node.')


if __name__ == '__main__':
    main()