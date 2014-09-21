#!/usr/bin/env python
#
# ball tracking code for FIDO
#

import roslib
roslib.load_manifest('fido')
import rospy
from fido.msg import MotorCommand

import cv2
from video import create_capture
import numpy as np
import time
from datetime import datetime

import pid
import servo_if

pos_x = 0
pos_y = 0

MAX_X = 640.0
MAX_Y = 480.0

CENTER_X = MAX_X / 2.0
CENTER_Y = MAX_Y / 2.0

HEAD_CENTER = 106
HEAD_RIGHT = HEAD_CENTER + 40
HEAD_LEFT = HEAD_CENTER - 40

NECK_UP = 230
NECK_DOWN = 100
NECK_CENTER = (NECK_UP + NECK_DOWN) / 2
NECK_START = (NECK_DOWN + NECK_CENTER) / 2

def get_thresholded_image(bgr_image):
    """return black & white image of parts of bgr_image that match 'tennis ball green'
    """
    hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
    thresholded_image = cv2.inRange(hsv_image, (30,70,128), (43, 144, 220))
    return thresholded_image


def set_motor_speed(speed_cmd_l, speed_cmd_r):
    """send a command to the motor_if ROS node to set left and right wheel motor speeds
    """
    pub.publish(MotorCommand(speed_cmd_l=speed_cmd_l, speed_cmd_r=speed_cmd_r))


def init_publisher():
    """initialize ROS publisher(s)"""
    global pub
    rospy.init_node('fido', anonymous=True)
    pub = rospy.Publisher('motor_command', MotorCommand)


def main():
    """track a tennis ball with FIDO's head and base"""
    global pos_x
    global pos_y

    init_publisher()
    #color_tracker_window = "output"
    #thresh_window = "thresh"
    #blob_window = "blob"
    cam = create_capture(-1)
    #cv2.namedWindow(color_tracker_window, 1)
    #cv2.moveWindow(color_tracker_window, 0, 0)
    #cv2.namedWindow(thresh_window, 1)
    #cv2.moveWindow(thresh_window, 700, 0)
    #cv2.namedWindow(blob_window, 1)
    #cv2.moveWindow(blob_window, 700, 500)

    # PIDs for head, neck, and base movement.
    # head & neck PID outputs are relative to center of view, so if the ball is centered, output = 0.
    head_x_pid = pid.PID(kP=0.020, kI=0.00075, kD=0.0, output_min=-HEAD_CENTER, output_max=HEAD_CENTER)
    head_x_pid.set_setpoint(CENTER_X)
    neck_y_pid = pid.PID(kP=0.020, kI=0.00075, kD=0.0, output_min=-NECK_CENTER, output_max=NECK_CENTER)
    neck_y_pid.set_setpoint(CENTER_Y)
    # base tracks the head and tries to move so that the head will be centered
    # output = rotational velocity (applied negatively to left wheel, positively to right)
    base_r_pid = pid.PID(kP=1.5, kI=0.05, kD=0.0, output_min=-250, output_max=250)
    base_r_pid.set_setpoint(HEAD_CENTER)

    base_area_pid = pid.PID(kP=0.02, kI=0.002, kD=0.0, output_min=-250, output_max=250)
    base_area_pid.set_setpoint(5500)

    servo_if.init_servos()
    time.sleep(0.25)
    servo_if.set_servo(servo_if.NECK, NECK_START)
    head_x = servo_if.get_servo_position(servo_if.HEAD)
    neck_y = servo_if.get_servo_position(servo_if.NECK)
    jaw_pos = servo_if.get_servo_position(servo_if.JAW)
    head_x_output = 0
    neck_y_output = 0
    base_x_output = 0
    
    last_known_x = None
    last_known_y = None
    
    frame_number = 0
    
    while True:
        start_time = time.time()
        frame_number += 1
        ret, frame = cam.read()
        #print "cam.read returned {0}".format(ret)
        if not ret:
            break
        #cv2.imshow(color_tracker_window, frame)
        #cv2.waitKey(1)
        blurred_image = cv2.GaussianBlur(frame, (9, 9), 0)
        thresholded_image = get_thresholded_image(blurred_image)
        #cv2.imshow(thresh_window, thresholded_image)
        #cv2.waitKey(1)
            
        #Calculating the moments
        #contours = None
        contours, hierarchy = cv2.findContours(thresholded_image,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        area = 0
        if contours:
            for c in contours:
                moments = cv2.moments(c)
                a = moments['m00']
                if a > area:
                    area = a
                    moment10 = moments['m10']
                    moment01 = moments['m01']

        lastX = pos_x
        lastY = pos_y

        #make sure we have a big enough blob
        if area > 100: 			
            #Calculate the coordinates of the centroid
            pos_x = int(moment10 / area)
            pos_y = int(moment01 / area)
            last_known_x = pos_x
            last_known_y = pos_y
        elif last_known_x is not None:
            if head_x > HEAD_LEFT and head_x < HEAD_RIGHT and neck_y > NECK_DOWN and neck_y < NECK_UP:
                pos_x = last_known_x
                pos_y = CENTER_Y
            else:
                #servo_if.ramp_servo(servo_if.HEAD, HEAD_CENTER, -3 if head_x > HEAD_CENTER else 3)
                #servo_if.ramp_servo(servo_if.NECK, NECK_START, -3 if neck_y > NECK_CENTER else 3)
                pos_x = CENTER_X
                pos_y = CENTER_Y
                last_known_x = None
                last_known_y = None


        if area > -1:
            head_x_output = head_x_pid.update(pos_x)
            head_x = min(max(head_x + head_x_output, HEAD_LEFT), HEAD_RIGHT)
            servo_if.set_servo(servo_if.HEAD, int(head_x))
        
            neck_y_output = neck_y_pid.update(pos_y)
            neck_y = min(max(neck_y - neck_y_output, NECK_DOWN), NECK_UP)             # for screen coordinates, +y = down, but for neck coordinates, +y = up
            servo_if.set_servo(servo_if.NECK, int(neck_y))
        
            base_x_output = base_r_pid.update(head_x)
            #base_x_output = 0
            base_area_output = base_area_pid.update(area) if area > 50 else 0
            left_motor_speed = min(max(base_x_output - base_area_output, -250), 250)
            right_motor_speed = min(max(-base_x_output - base_area_output, -250), 250)
            set_motor_speed(int(left_motor_speed), int(right_motor_speed))  # for +output, left motor goes forward, right motor goes backward
        
            jaw_pos = int((float(area) - 1000.0) / 20000.0 * (servo_if.JAW_OPEN - servo_if.JAW_CLOSED_EMPTY) + servo_if.JAW_CLOSED_EMPTY)
            jaw_pos = max(min(jaw_pos, servo_if.JAW_OPEN), servo_if.JAW_CLOSED_EMPTY)
            servo_if.set_servo(servo_if.JAW, jaw_pos)

        end_time = time.time()
        if not frame_number % 10:
            print '{}. {} x: {} y: {} area: {} head: {} {} neck: {} {} base: {} {} jaw_pos: {} elapsed: {}'.format(frame_number, datetime.now(), pos_x, pos_y, area, head_x, head_x_output, neck_y, neck_y_output, base_x_output, base_area_output, jaw_pos, end_time - start_time)
        #cv2.imshow(blob_window, thresholded_image)
        time.sleep(0.001)
        #c = cv2.waitKey(3)
        #if(c!=-1):
        #    break
    print "exiting"
    #cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
