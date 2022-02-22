#!/usr/bin/python

import rospy
import serial
import time
from control_input_aggregator.msg import ControlInput
from embedded_controller_relay.msg import NavSatReport, BatteryReport
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from std_msgs import *

# Serial Port Reference
ser = None

# Publishers for Sensor Data
gps_native_pub = None
gps_pub = None
battery_pub = None
status_pub = None

# Subscribers for Control Data
control_input_sub = None
cmd_vel_sub = None


def process_gps(data):
    # initialize message objects
    navSatFix = NavSatFix()
    navSatReport = NavSatReport()

    # parse input data into keys for reference
    args = data.split(',')
    results = {}
    for pair in args:
        name, value = pair.split('=')
        results[name] = value
    
    # interpret data and store into message
    navSatReport.timestamp = results['time']
    navSatReport.latitude = int(results['lat']) / 10000000.0
    navSatReport.longitude = int(results['long']) / 10000000.0
    navSatReport.altitude = int(results['alt']) / 1000.0
    navSatReport.ground_speed = int(results['ground_speed']) / 1000.0
    navSatReport.motion_heading = int(results['motion_heading']) / 100000.0
    navSatReport.horizontal_accuracy = int(results['horizontal_accuracy']) / 1000.0

    # copy data over to the native message type
    navSatFix.latitude = navSatReport.latitude
    navSatFix.longitude = navSatReport.longitude
    navSatFix.altitude = navSatReport.altitude

    gps_native_pub.publish(navSatFix)
    gps_pub.publish(navSatReport)


def process_battery(data):
    # initialize message objects
    batteryReport = BatteryReport()

    # parse input data into keys for reference
    args = data.split(',')
    results = {}
    for pair in args:
        name, value = pair.split('=')
        results[name] = value

    batteryReport.batteryVoltage = float(results['v'])
    batteryReport.batteryCharge = float(results['c'])

    battery_pub.publish(batteryReport)

def process_status(data):
    status_pub.publish(str(data))


topic_publisher_callback = {
    'gps' : process_gps,
    'battery' : process_battery,
    'status' : process_status,
    'response': process_status
}
def process_message(message):
    args = message.split(";")
    topic = args[0]
    data = args[1]

    topic_publisher_callback[topic](data)


def handle_control_input(controlInput):
    ser.write("set_motors;" + str(controlInput.heading[0]) + "," + str(controlInput.heading[1]) + "," + str(controlInput.speed_clamp) + "\n")

def handle_vel_cmd(vel_cmd):
    ser.write("set_motors;" + str(vel_cmd.linear.x) + ',' + str(-vel_cmd.angular.z) + ',' + "1\n")

def run():
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        while ser.in_waiting > 0:
            message = ser.read_until('\n')
            #process_message(message)
        
        #ser.write("set_motors;1.0,0.5\n")
        rate.sleep()


def main():
    global gps_native_pub, gps_pub, ser, status_pub, battery_pub, control_input_sub
    rospy.init_node('embedded_controller_relay')

    while (ser is None):
        try:
            ser = serial.Serial('/dev/ttyACM0')
        except:
            rospy.loginfo('Unable to locate Teensy, please ensure communications.')
            time.sleep(3)
    rospy.loginfo('Connected to teensy.')

    # Initilize Publishers
    #gps_native_pub = rospy.Publisher('gps_native', NavSatFix, queue_size=5)
    #gps_pub = rospy.Publisher('gps', NavSatReport, queue_size=5)

    #status_pub = rospy.Publisher('status', msg.String, queue_size=5)

    #battery_pub = rospy.Publisher('battery', BatteryReport, queue_size=1)
    
    # Initialize Subscribers
    #control_input_sub = rospy.Subscriber("/control_output", ControlInput, handle_control_input)
    cmd_vel_sub = rospy.Subscriber("/planner/cmd_vel", Twist, handle_vel_cmd)


    # Function to continullay relay data between the Jetson and Teensy
    run()

    # End of Run tasks
    rospy.loginfo('Disconnecting teensy.')
    ser.close()
    rospy.loginfo('Shutting down node.')


if __name__ == '__main__':
    main()
