#!/usr/bin/env python
"""motor_if.py - read serial data from Arduino motor controller and publish encoder counts, speeds, and motor command feedback
"""

import roslib; roslib.load_manifest('fido')
import rospy
from fido.msg import MotorStatus, MotorCommand
import serial
import string
import sys

sonar_l = 0.0
sonar_r = 0.0
encoder_l = 0.0
encoder_r = 0.0
speed_cmd_l = 0.0
speed_cmd_r = 0.0
speed_act_l = 0.0
speed_act_r = 0.0
out_l = 0.0
out_r = 0.0

def read_motor_status():
    global sonar_l
    global sonar_r
    global encoder_l
    global encoder_r
    global speed_cmd_l
    global speed_cmd_r
    global speed_act_l
    global speed_act_r
    global out_l
    global out_r

    mot_line = mot.readline()
    #rospy.loginfo(mot_line)
    words = string.split(mot_line,",")
    
    if len(words) >= 9:
        try:
            sonar_l = float(words[1])
            sonar_r = float(words[2])
            encoder_l = float(words[3])
            encoder_r = float(words[4])
            speed_cmd_l = float(words[5])
            speed_cmd_r = float(words[6])
            speed_act_l = float(words[7])
            speed_act_r = float(words[8])
            out_l = float(words[9])
            out_r = float(words[10])
        except:
            rospy.loginfo('bad line')
    else:
        rospy.loginfo('line too short')


def talker():
    pub = rospy.Publisher('motor_data', MotorStatus)
    rospy.init_node('motor_if')
    while not rospy.is_shutdown():
        read_motor_status()
        #print encoder_l, encoder_r
        pub.publish(MotorStatus(sonar_l=sonar_l, sonar_r=sonar_r,
                                encoder_l=encoder_l, encoder_r=encoder_r,
                                speed_cmd_l=speed_cmd_l, speed_cmd_r=speed_cmd_r,
                                speed_act_l=speed_act_l, speed_act_r=speed_act_r,
                                out_l=out_l, out_r=out_r))
        #rospy.sleep(0.2)

def do_motor_command(data):
    mot.write('{0}l{1}r'.format(data.speed_cmd_l, data.speed_cmd_r))


def listener():
    rospy.Subscriber('motor_command', MotorCommand, do_motor_command)

if __name__ == '__main__':
    argv = sys.argv
    if len(argv) > 1:
        port = argv[1]
    else:
        port = '/dev/serial/by-id/usb-Arduino_LLC_Arduino_Leonardo-if00'

    mot = serial.Serial(port=port, baudrate=57600, timeout=1)
    try:
        listener()
        talker()
    except rospy.ROSInterruptException: pass
