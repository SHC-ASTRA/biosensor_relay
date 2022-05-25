#!/usr/bin/python

import rospy
import serial
import time
from bio_relay.srv import BioCommand, BioCommandResponse
from std_msgs import *

class BioRelay:
    def __init__(self):
        rospy.init_node('bio_relay')
        rospy.loginfo("Initializing biosensor relay.")

        # Initialize serial port and connect to teensy microcontroller
        self.ser = None
        while (self.ser is None):
            port = rospy.get_param('bio_serial_port', '/dev/ttyACM0')
            try:
                self.ser = serial.Serial(port=port,baudrate=115200,timeout=0.1)
            except KeyboardInterrupt:
                exit()
            except:
                rospy.loginfo("Unable to locate Teensy (@{0}), please ensure communications.".format(port))
                time.sleep(3)
        rospy.loginfo('Connected to teensy.')
 
        # Initialize Publisher Nodes for Sensor Data

        self.status_pub = rospy.Publisher('status', msg.String, queue_size=10)

        self.command_srv = rospy.Service("bio_command", BioCommand, self.send_command)

    def process_status(self, data):
        self.status_pub.publish(str(data))

    def process_message(self, message):
        self.process_status(message)

    def send_command(self, srv):
        command = srv.command
        if len(command) <= 2:
            command += " 0"
        self.ser.write(command+"\n")
    
        return BioCommandResponse(True)

    def run(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            while self.ser.in_waiting > 0:
                message = self.ser.read_until('\n')
                print(message)
                self.process_message(message)
            
            rate.sleep()

def main():
    relay = BioRelay()
    relay.run()


if __name__ == '__main__':
    main()
