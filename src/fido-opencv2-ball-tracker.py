#!/usr/bin/env python
#
# ball tracking code for FIDO
#

import roslib
roslib.load_manifest('fido')
import rospy
from nomad1.msg import MotorCommand

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
    # PID outputs are relative to center of view, so if the ball is centered, output = 0.
    head_x_pid = pid.PID(kP=0.25, kI=0.0, kD=0.0, output_min=-HEAD_CENTER, output_max=HEAD_CENTER)
    neck_y_pid = pid.PID(kP=0.125, kI=0.0, kD=0.0, output_min=-NECK_CENTER, output_max=NECK_CENTER)
    # for the base, output = rotational velocity (applied negatively to left wheel, positively to right)
    base_x_pid = pid.PID(kP=0.1, kI=0.0, kD=0.0, output_min=-100, output_max=100)

    servo_if.init_servos()
    time.sleep(0.25)
    servo_if.set_servo(servo_if.NECK, NECK_START)
    head_x = servo_if.get_servo_position(servo_if.HEAD)
    neck_y = servo_if.get_servo_position(servo_if.NECK)
    jaw_pos = servo_if.get_servo_position(servo_if.JAW)
    
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
                pos_y = last_known_y
            else:
                servo_if.ramp_servo(servo_if.HEAD, HEAD_CENTER, -3 if head_x > HEAD_CENTER else 3)
                servo_if.ramp_servo(servo_if.NECK, NECK_START, -3 if neck_y > NECK_CENTER else 3)
                pos_x = CENTER_X
                pos_y = CENTER_Y
                last_known_x = None
                last_known_y = None

        print str(datetime.now()), ' x: ' + str(pos_x) + ' y: ' + str(pos_y) + ' area: ' + str(area) + ' head_x: ' + str(head_x) + ' neck_y: ' + str(neck_y) + ' jaw_pos: ' + str(jaw_pos)
        #drawing lines to track the movement of the blob
        if(lastX > 0 and lastY > 0 and pos_x > 0 and pos_y > 0):
            #cv.Circle( thresholded_image, (pos_x, pos_y), maxRadius, cv.Scalar(0,0,255), 3, 8, 0 );        
            #cv.Line(imgScrible, (pos_x, pos_y), (lastX, lastY), cv.Scalar(0, 0, 255), 5)
            if pos_x < CENTER_X - 10:
                error_x = (pos_x - CENTER_X) / MAX_X * (HEAD_RIGHT - HEAD_LEFT)
                desired_x = int(error_x) / 4 + head_x
                head_x = max(desired_x, HEAD_LEFT)
                servo_if.set_servo(servo_if.HEAD, head_x)
            elif pos_x > CENTER_X + 10:
                new_x = (pos_x - CENTER_X) / MAX_X * (HEAD_RIGHT - HEAD_LEFT)
                head_x = min(int(new_x) / 4 + head_x, HEAD_RIGHT)
                servo_if.set_servo(servo_if.HEAD, head_x)

            if pos_y < CENTER_Y - 10:
                new_y = (pos_y - CENTER_Y) / MAX_Y * (NECK_UP - NECK_DOWN)
                neck_y = min(neck_y - (int(new_y) / 8), NECK_UP)
                servo_if.set_servo(servo_if.NECK, neck_y)
            elif pos_y > CENTER_Y + 10:
                new_y = (pos_y - CENTER_Y) / MAX_Y * (NECK_UP - NECK_DOWN)
                neck_y = max(neck_y - (int(new_y) / 8), NECK_DOWN)
                servo_if.set_servo(servo_if.NECK, neck_y)

            jaw_pos =int((float(area) - 1000.0) / 25000.0 * (servo_if.JAW_OPEN - servo_if.JAW_CLOSED_EMPTY) + servo_if.JAW_CLOSED_EMPTY)
            jaw_pos = max(min(jaw_pos, servo_if.JAW_OPEN), servo_if.JAW_CLOSED_EMPTY)
            servo_if.set_servo(servo_if.JAW, jaw_pos)

        #cv2.imshow(blob_window, thresholded_image)
        end_time = time.time()
        print "elapsed time = {}".format(end_time - start_time)
        #c = cv2.waitKey(3)
        #if(c!=-1):
        #  break
    print "exiting"
    #cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
