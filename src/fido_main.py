#!/usr/bin/env python
#
# ball tracking code for FIDO
#

import roslib
roslib.load_manifest('fido')
import rospy
from fido.msg import MotorCommand, MotorStatus
import cv2
from video import create_capture
import numpy as np
import time
from datetime import datetime
import pid
import servo_if
import fido_fsm
import math
import sys
import getopt
import signal


#
# CONSTANTS
#

MAX_X = 640.0
MAX_Y = 480.0

SCREEN_AREA = MAX_X * MAX_Y

CENTER_X = MAX_X / 2.0
CENTER_Y = MAX_Y / 2.0

AREA_MOVING_AVG_POINTS = 10

JAW_CLOSE_AREA = SCREEN_AREA / 640
JAW_OPEN_AREA = SCREEN_AREA / 64

TAIL_START_WAG_AREA = SCREEN_AREA / 512
TAIL_STOP_WAG_AREA = SCREEN_AREA / 640

BALL_MIN_HSV = (30,90,120)
BALL_MAX_HSV = (85,180,230)



#
# GLOBAL VARS
#

debug = False
quit_request = False

ir_l = 0.0
ir_m = 0.0
ir_r = 0.0
encoder_l = 0.0
encoder_r = 0.0
speed_cmd_l = 0.0
speed_cmd_r = 0.0
speed_act_l = 0.0
speed_act_r = 0.0
out_l = 0.0
out_r = 0.0
batt_v = 0.0

cascade = None
nested = None


# FSM clients
cam = None
brain = None
tail = None

# PID loops
head_x_pid = None
neck_y_pid = None
base_r_pid = None
base_area_pid = None

# current input values from all sensors (and values computed from them)
inputs = {'cam_status': None,
          'cam_frame': None,
          'ball_x': 0,
          'ball_y': 0,
          'ball_area': 0,
          'avg_ball_area': 0,
          'last_known_ball_x': CENTER_X,
          'last_known_ball_y': CENTER_Y,
          'face_x': 0,
          'face_y': 0,
          'face_area': 0,
          'last_known_face_x': CENTER_X,
          'last_known_face_y': MAX_Y
}

# current output values to all controlled devices
outputs = {'left_wheel': 0,
           'right_wheel': 0,
           'legs': 0,
           'neck': 0,
           'head': 0,
           'jaw': 0,
           'tail': 0}

# previous output values to all controlled devices
prev_outputs = {'left_wheel': 0,
                'right_wheel': 0,
                'legs': 0,
                'neck': 0,
                'head': 0,
                'jaw': 0,
                'tail': 0}


x_co = 160
y_co = 120
def on_mouse(event,x,y,flag,param):
  global x_co
  global y_co
  if(event==cv2.cv.CV_EVENT_MOUSEMOVE):
    x_co=x
    y_co=y

    
def get_ball_image(bgr_image):
    """return black & white image of parts of bgr_image that match 'tennis ball green'
    """
    blurred_image = cv2.GaussianBlur(bgr_image, (9, 9), 0)
    hsv_image = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2HSV)
    inputs['hsv_image'] = hsv_image
    thresholded_image = cv2.inRange(hsv_image, BALL_MIN_HSV, BALL_MAX_HSV)
    return thresholded_image


def find_ball():
    """locate a tennis ball within the current camera frame"""
    ball_image = get_ball_image(inputs['cam_frame'])
            
    #Calculate moments
    contours, hierarchy = cv2.findContours(ball_image,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    area = 0
    pos_x = CENTER_X
    pos_y = CENTER_Y

    if contours:
        for c in contours:
            moments = cv2.moments(c)
            a = moments['m00']
            if a > area:
                area = a
                moment10 = moments['m10']
                moment01 = moments['m01']
                pos_x = int(moment10 / area)
                pos_y = int(moment01 / area)
                size = math.sqrt(area) / 2.0
                if debug:
                    cv2.rectangle(inputs['cam_frame'], (int(pos_x - size), int(pos_y - size)), (int(pos_x + size), int(pos_y + size)), (255, 0, 0), 2)
                
    #make sure we have a big enough blob
    if area > 100: 			
        #Calculate the coordinates of the centroid
        pos_x = int(moment10 / area)
        pos_y = int(moment01 / area)
        inputs['last_known_ball_x'] = pos_x
        inputs['last_known_ball_y'] = pos_y
    elif inputs['last_known_ball_x'] is not None:
        if outputs['head'] > servo_if.HEAD_LEFT and outputs['head'] < servo_if.HEAD_RIGHT and outputs['neck'] > servo_if.NECK_DOWN and outputs['neck'] < servo_if.NECK_UP:
            pos_x = inputs['last_known_ball_x']
            pos_y = CENTER_Y
            
    inputs['ball_x'] = pos_x
    inputs['ball_y'] = pos_y
    inputs['ball_area'] = area
    inputs['avg_ball_area'] = (inputs['avg_ball_area'] * (AREA_MOVING_AVG_POINTS - 1) + area) / AREA_MOVING_AVG_POINTS
       


def detect_faces(img, cascade):
    rects = cascade.detectMultiScale(img, scaleFactor=1.3, minNeighbors=4, minSize=(30, 30), flags = cv2.cv.CV_HAAR_SCALE_IMAGE)
    if len(rects) == 0:
        return []
    rects[:,2:] += rects[:,:2]
    return rects


def find_face():
    """locate a face within the current camera frame"""
    gray = cv2.cvtColor(inputs['cam_frame'], cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)
    rects = detect_faces(gray, cascade)

    area = 0
    pos_x = CENTER_X
    pos_y = CENTER_Y
    
    for x1, y1, x2, y2 in rects:
        a = (x2 - x1) * (y2 - y1)
        if a > area:
            area = a
            pos_x = (x1 + x2) / 2
            pos_y = (y1 + y2) / 2
            size = math.sqrt(area) / 2.0
            if debug:
                cv2.rectangle(inputs['cam_frame'], (int(pos_x - size), int(pos_y - size)), (int(pos_x + size), int(pos_y + size)), (255, 255, 0), 2)
 
    if area > 100: 			
        inputs['last_known_face_x'] = pos_x
        inputs['last_known_face_y'] = pos_y
    elif inputs['last_known_face_x'] is not None:
        if outputs['head'] > servo_if.HEAD_LEFT and outputs['head'] < servo_if.HEAD_RIGHT and outputs['neck'] > servo_if.NECK_DOWN and outputs['neck'] < servo_if.NECK_UP:
            pos_x = inputs['last_known_face_x']
            pos_y = CENTER_Y

    inputs['face_x'] = pos_x
    inputs['face_y'] = pos_y
    inputs['face_area'] = area

    
def init_pids():
    """PIDs for head, neck, and base movement"""
    global head_x_pid, neck_y_pid, base_r_pid, base_area_pid
    # head & neck PID outputs are relative to center of view, so if the ball is centered, output = 0.
    head_x_pid = pid.PID(kP=0.020, kI=0.002, kD=0.0, output_min=-servo_if.HEAD_CENTER, output_max=servo_if.HEAD_CENTER)
    head_x_pid.set_setpoint(CENTER_X)
    neck_y_pid = pid.PID(kP=0.020, kI=0.001, kD=0.0, output_min=-servo_if.NECK_CENTER, output_max=servo_if.NECK_CENTER)
    neck_y_pid.set_setpoint(CENTER_Y)
    # base tracks the head and tries to move so that the head will be centered
    # output = rotational velocity (applied negatively to left wheel, positively to right)
    base_r_pid = pid.PID(kP=1.0, kI=0.05, kD=0.0, output_min=-1000, output_max=1000)
    base_r_pid.set_setpoint(servo_if.HEAD_CENTER)

    base_area_pid = pid.PID(kP=0.01, kI=0.0005, kD=0.0, output_min=-1000, output_max=1000)
    base_area_pid.set_setpoint(18000)

    
def init_subscriptions():
    rospy.Subscriber('motor_data', MotorStatus, update_motor_status)

    
def init_publisher():
    """initialize ROS publisher(s)"""
    global pub
    rospy.init_node('fido', anonymous=True)
    pub = rospy.Publisher('motor_command', MotorCommand)

    
def handle_SIGTERM(arg1, arg2):
    global quit_request
    print("caught SIGTERM, exiting")
    quit_request = True
    
def handle_SIGINT(arg1, arg2):
    global quit_request
    print("caught SIGINT, exiting")
    quit_request = True

    
def init_sighandler():
    signal.signal(signal.SIGTERM, handle_SIGTERM)
    signal.signal(signal.SIGINT, handle_SIGINT)


def update_pids():

    if brain.state() in ['TrackingBall', 'ApproachingBall', 'FinalApproach']:
        pos_x = inputs['ball_x']
        pos_y = inputs['ball_y']
        area = inputs['ball_area']
    elif brain.state() in ['ApproachingFace']:
        pos_x = inputs['face_x']
        pos_y = inputs['face_y']
        area = inputs['face_area']
    else:
        # not in a state where we want to update PIDs...
        tail.stop_wagging()
        return
        
    head_x_output = head_x_pid.update(pos_x)
    head_x = min(max(outputs['head'] + head_x_output, servo_if.HEAD_LEFT), servo_if.HEAD_RIGHT)
    outputs['head'] = head_x
        
    neck_y_output = neck_y_pid.update(pos_y)
    # for screen coordinates, +y = down, but for neck coordinates, +y = up
    neck_y = min(max(outputs['neck'] - neck_y_output, (servo_if.NECK_CENTER if brain.should_find_face() else servo_if.NECK_DOWN)), servo_if.NECK_UP)
    outputs['neck'] = neck_y

    if brain.state() in ['ApproachingBall', 'ApproachingFace']:
        if abs(prev_outputs['head'] - servo_if.HEAD_CENTER) < 5:
            base_x_output = 0
        else:
            base_x_output = base_r_pid.update(prev_outputs['head'])
            
        base_area_output = base_area_pid.update(area) if area > 50 else 0
        left_motor_speed = min(max(base_x_output - base_area_output, -1000), 1000)
        right_motor_speed = min(max(-base_x_output - base_area_output, -1000), 1000)
        outputs['left_wheel'] = left_motor_speed
        outputs['right_wheel'] = right_motor_speed
    else:
        outputs['left_wheel'] = 0
        outputs['right_wheel'] = 0
        
    jaw_pos = int((float(inputs['ball_area']) - JAW_CLOSE_AREA) / JAW_OPEN_AREA * (servo_if.JAW_OPEN - servo_if.JAW_CLOSED_EMPTY) + servo_if.JAW_CLOSED_EMPTY)
    jaw_pos = max(min(jaw_pos, servo_if.JAW_OPEN), servo_if.JAW_CLOSED_EMPTY)
    outputs['jaw'] = jaw_pos

    if area > TAIL_START_WAG_AREA:
        if not tail.is_wagging():
            tail.start_wagging(0.25, 9999)
    elif area < TAIL_STOP_WAG_AREA:
        if tail.is_wagging():
            tail.stop_wagging()

    
def update_motor_status(data):
    global ir_l
    global ir_m
    global ir_r
    global encoder_l
    global encoder_r
    global speed_cmd_l
    global speed_cmd_r
    global speed_act_l
    global speed_act_r
    global out_l
    global out_r
    global batt_v

    ir_l = data.ir_l
    ir_m = data.ir_m
    ir_r = data.ir_r
    encoder_l = data.encoder_l
    #rospy.loginfo(str(encoder_l))
    encoder_r = data.encoder_r
    speed_cmd_l = data.speed_cmd_l
    speed_cmd_r = data.speed_cmd_r
    speed_act_l = data.speed_act_l
    speed_act_r = data.speed_act_r
    out_l = data.out_l
    out_r = data.out_r
    batt_v = data.batt_v
    #rospy.loginfo('update motor status')

    
def set_motor_speed(speed_cmd_l, speed_cmd_r):
    """send a command to the motor_if ROS node to set left and right wheel motor speeds
    """
    pub.publish(MotorCommand(speed_cmd_l=speed_cmd_l, speed_cmd_r=speed_cmd_r))


def read_inputs():
    """read all sensors"""
    global inputs
    inputs['ir_l'] = ir_l
    inputs['ir_m'] = ir_m
    inputs['ir_r'] = ir_r
    inputs['cam_status'], inputs['cam_frame'] = cam.read()
    if inputs['cam_status']:
        if brain.should_find_ball():
            find_ball()
        else:
            inputs['ball_area'] = 0
            inputs['avg_ball_area'] = 0
        if brain.should_find_face():
            find_face()
        else:
            inputs['face_area'] = 0

    
def write_outputs():
    """write current output values to motors & servos"""
    global outputs, prev_outputs
    #t1 = time.time()

    if outputs['left_wheel'] != prev_outputs['left_wheel'] or outputs['right_wheel'] != prev_outputs['right_wheel']:
        set_motor_speed(int(outputs['left_wheel']), int(outputs['right_wheel']))
        prev_outputs['left_wheel'] = outputs['left_wheel']
        prev_outputs['right_wheel'] = outputs['right_wheel']

    if outputs['legs'] != prev_outputs['legs']:
        servo_if.set_legs(int(outputs['legs']))
        prev_outputs['legs'] = outputs['legs']

    if outputs['neck'] != prev_outputs['neck']:
        servo_if.set_servo(servo_if.NECK, int(outputs['neck']))
        prev_outputs['neck'] = outputs['neck']

    if outputs['head'] != prev_outputs['head']:
        servo_if.set_servo(servo_if.HEAD, int(outputs['head']))
        prev_outputs['head'] = outputs['head']

    if outputs['jaw'] != prev_outputs['jaw']:
        servo_if.set_servo(servo_if.JAW, int(outputs['jaw']))
        prev_outputs['jaw'] = outputs['jaw']

    if outputs['tail'] != prev_outputs['tail']:
        servo_if.set_servo_reliably(servo_if.TAIL, int(outputs['tail']))
        prev_outputs['tail'] = outputs['tail']
        
    #t2 = time.time()
    #print 'output time = {}'.format(t2 - t1)

        
def main():
    """track a tennis ball with FIDO's head and base"""
    global cam
    global brain, tail
    global inputs, outputs, prev_outputs
    global cascade, nested
    global debug, quit_request

    try:
        opts, args = getopt.gnu_getopt(sys.argv, "hd", ["help", "debug"])
    except getopt.GetoptError as err:
        print err
        print 'fido_main.py [--help] [--debug]'
        sys.exit(2)

    print "opts = ", opts
    print "args = ", args
        
    for opt, arg in opts:
        print "opt = ", opt
        if opt in ('-h', '--help'):
            print 'fido_main.py [--help] [--debug]'
            sys.exit()
        elif opt in ('-d', '--debug'):
            debug = True

    print "debug = {}".format(debug)
    
    cascade_fn = "../data/haarcascades/haarcascade_frontalface_alt.xml"
    #nested_fn  = "../data/haarcascades/haarcascade_eye.xml"

    cascade = cv2.CascadeClassifier(cascade_fn)
    #nested = cv2.CascadeClassifier(nested_fn)
    
    cam = create_capture(-1)
    cam.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, MAX_Y)
    cam.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, MAX_X)

    if debug:
        cv2.namedWindow("output", 1)
        cv2.moveWindow("output", 0, 540)
    
    init_pids()
    init_publisher()
    init_subscriptions()
    init_sighandler()
    
    servo_if.init_servos()
    time.sleep(0.25)
    servo_if.set_servo(servo_if.NECK, servo_if.NECK_START)
    outputs['legs'] = servo_if.legs_position
    outputs['head'] = servo_if.get_servo_position(servo_if.HEAD)
    outputs['neck'] = servo_if.get_servo_position(servo_if.NECK)
    outputs['jaw'] = servo_if.get_servo_position(servo_if.JAW)
    outputs['tail'] = servo_if.get_servo_position(servo_if.TAIL)

    prev_outputs['left_wheel'] = outputs['left_wheel']
    prev_outputs['right_wheel'] = outputs['right_wheel']
    prev_outputs['legs'] = outputs['legs']
    prev_outputs['neck'] = outputs['neck']
    prev_outputs['head'] = outputs['head']
    prev_outputs['jaw'] = outputs['jaw']
    prev_outputs['tail'] = outputs['tail']
    
    brain = fido_fsm.FidoBrain(inputs, outputs)
    tail = fido_fsm.FidoTail(inputs, outputs)

    frame_number = 0
    
    if debug:
        cv2.cv.SetMouseCallback("output", on_mouse, 0)

    while not quit_request:
        start_time = time.time()
        frame_number += 1
        read_inputs()
        if not inputs['cam_status']:
            print "camera read failure"
            break
        update_pids()
        brain.run() 
        tail.run()
        write_outputs()
        if debug:
            s = inputs['hsv_image'][y_co, x_co]
            cv2.putText(inputs['cam_frame'], str(s[0])+","+str(s[1])+","+str(s[2]), (x_co,y_co), cv2.FONT_HERSHEY_SIMPLEX, 1, 255)
            cv2.imshow("output", inputs['cam_frame'])
        end_time = time.time()
        if True: #not frame_number % 10:
            print '{}. {} v: {} ir: {} {} {} x: {} y: {} area: {} {} brain: {} tail: {} head: {} neck: {} base: {} {} jaw: {} elapsed: {}'.format(frame_number, datetime.now(), batt_v, inputs['ir_l'], inputs['ir_m'], inputs['ir_r'], inputs['ball_x'], inputs['ball_y'], inputs['avg_ball_area'], inputs['face_area'], brain.state(), tail.state(), outputs['head'], outputs['neck'], outputs['left_wheel'], outputs['right_wheel'], outputs['jaw'], end_time - start_time)
        k = cv2.waitKey(10)
        if k > 0:
            print "key pressed, exiting"
            exit(1)
        time.sleep(max(0.05 - (time.time() - start_time), 0.001))


if __name__ == "__main__":
    main()
