#!/usr/bin/python

import rospy
import serial
import time
from control_input_aggregator.msg import ControlInput
from embedded_controller_relay.msg import NavSatReport, BatteryReport
from embedded_controller_relay.srv import SignalColor, SignalColorResponse
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from std_msgs import *

class EmbeddedControllerRelay:
    def __init__(self):
        rospy.init_node('embedded_controller_relay')

        # Initialize serial port and connect to teensy microcontroller
        self.ser = None
        while (self.ser is None):
            port = rospy.get_param('~teensy_serial_port', '/dev/ttyACM0')
            try:
                self.ser = serial.Serial(port)
            except KeyboardInterrupt:
                exit()
            except:
                rospy.loginfo("Unable to locate Teensy (@{0}), please ensure communications.".format(port))
                time.sleep(3)
        rospy.loginfo('Connected to teensy.')

        # Initialize Publisher Nodes for Sensor Data
        self.gps_native_pub = rospy.Publisher('gps_native', NavSatFix, queue_size=1)
        self.gps_pub = rospy.Publisher('gps', NavSatReport, queue_size=1)

        self.status_pub = rospy.Publisher('status', msg.String, queue_size=10)
        self.battery_pub = rospy.Publisher('battery_status', BatteryReport, queue_size=1)

        # Initialize Subscriber Nodes for Control Data
        self.cmd_vel_sub = rospy.Subscriber("/planner/cmd_vel", Twist, self.process_vel_cmd)
        self.control_input_sub = rospy.Subscriber("/control_input", ControlInput, self.process_control_input)

        # Signal Service
        self.signal_srv = rospy.Service("/signal_operating_mode", SignalColor, self.signal_color)

        self.topic_publisher_callback = {
            'gps' : self.process_gps,
            'battery' : self.process_battery,
            'status' : self.process_status,
            'response': self.process_status
        }

    def process_vel_cmd(self, vel_cmd):
        self.ser.write("set_motors;" + str(-vel_cmd.linear.x) + ',' + str(-vel_cmd.angular.z) + ',' + "1\n")

    def process_control_input(self, input):
        self.ser.write("set_motors;" + str(input.heading[0]) + "," + str(-input.heading[1]) + "," + str(input.speed_clamp) + "\n")

    def process_gps(self, data):
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
        navSatReport.latitude = int(results['lat'])
        navSatReport.longitude = int(results['long'])
        navSatReport.altitude = int(results['alt'])
        navSatReport.ground_speed = int(results['ground_speed'])
        navSatReport.motion_heading = int(results['motion_heading'])
        navSatReport.horizontal_accuracy = int(results['horizontal_accuracy'])

        # copy data over to the native message type
        navSatFix.latitude = navSatReport.latitude
        navSatFix.longitude = navSatReport.longitude
        navSatFix.altitude = navSatReport.altitude

        self.gps_native_pub.publish(navSatFix)
        self.gps_pub.publish(navSatReport)

    def process_battery(self, data):
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

        self.battery_pub.publish(batteryReport)

    def process_status(self, data):
        self.status_pub.publish(str(data))

    def process_message(self, message):
        try:
            args = message.split(";")
            topic = args[0]
            data = args[1]

            self.topic_publisher_callback[topic](data)
        except:
            pass

    def signal_color(self, srv):
        if srv.signal == "teleoperation":
            self.ser.write("signal_teleop;\n")
        elif srv.signal == "autonomous":
            self.ser.write("signal_autonomous;\n")
        elif srv.signal == "goal":
            self.ser.write("signal_goal;\n")
        elif srv.signal == "idle":
            self.ser.write("signal_idle;\n")

        return SignalColorResponse(True)

    def run(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            while self.ser.in_waiting > 0:
                message = self.ser.read_until('\n')
                print(message)
                self.process_message(message)
            
            rate.sleep()

def main():
    relay = EmbeddedControllerRelay()
    relay.run()


if __name__ == '__main__':
    main()
