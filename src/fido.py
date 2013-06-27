#!/usr/bin/python

import serial
import time

# Servo numbers
LEFT_WHEELS = 1
RIGHT_WHEELS = 2
LEFT_LEG = 3    # left and right legs must move synchronized, in opposite directions!
RIGHT_LEG = 4
NECK = 5
HEAD = 6
JAW = 7
TAIL = 8

# Servo positions
LEGS_DOWN = 20 # -15  # relative to leg center - use only with set_legs()
LEGS_UP = 73     # relative to leg center - use only with set_legs()
NECK_DOWN = 84
NECK_UP = 170
HEAD_LEFT = 0x6A-0x25
HEAD_CENTER = 0x6A
HEAD_RIGHT = 0x6A+0x25
JAW_OPEN = 160
JAW_CLOSED_FULL = 85
JAW_CLOSED_EMPTY = 65
TAIL_LEFT = 136-50
TAIL_CENTER = 136
TAIL_RIGHT = 136+50

port = serial.Serial("/dev/ttyUSB0", 19200, timeout=0.1)
legs_position = 0

def get_legs_position():
    return legs_position

def get_servo_position(servo_number=1):
    port.write(">1{0}g".format(servo_number))
    response = port.read(2)
    if len(response) != 2:
        print "get_servo_position: bad response: {0}".format(repr(response))
        return -1
    return ord(response[0])

def set_servo(servo_number=1, position=0):
    port.write(">1{0}a{1}".format(servo_number, chr(position)))
    ack = port.read()
    if ack != '\r':
        print "set_servo: bad ack = {0}\n".format(repr(ack))

def set_servo_reliably(servo_number=1, position=0):
    set_servo(servo_number, position)
    while get_servo_position(servo_number) != position:
        set_servo(servo_number, position)

def set_all_servos(pos1=0,pos2=0,pos3=0,pos4=0,pos5=0,pos6=0,pos7=0,pos8=0):
    port.write(">11m{0}{1}{2}{3}{4}{5}{6}{7}".format(chr(pos1), chr(pos2), chr(pos3), chr(pos4),
                                                     chr(pos5), chr(pos6), chr(pos7), chr(pos8)))
    ack = port.read()
    if ack != '\r':
        print "set_all_servos: bad ack = {0}\n".format(repr(ack))

def set_legs(position=0):
    global legs_position
    for i in xrange(3):
        set_servo(LEFT_LEG, 134+position)
        set_servo(RIGHT_LEG, 130-position)
    legs_position = position

def ramp_legs(position=0, step=1, delay=0.02):
    global legs_position
    for i in range(legs_position, position, step):
        set_legs(i)
        time.sleep(delay)
    set_legs(position)
    
def stop():
    set_all_servos(0,0,0,0,0,0,0,0)

def ramp_servo(servo_number=0, position=0, step=1, delay=0.02):
    start_pos = get_servo_position(servo_number)
    while start_pos == -1:
        start_pos = get_servo_position(servo_number)
    for i in range(start_pos, position, step):
        set_servo(servo_number, i)
        time.sleep(delay)
    set_servo_reliably(servo_number, position)

def wag(servo, left, right, center, times=1,delay=0.3):
    for i in xrange(times):
        set_servo_reliably(servo, right)
        time.sleep(delay)
        set_servo_reliably(servo, left)
        time.sleep(delay)
    set_servo(servo, center)

def slow_wag(servo, left, right, center, rate, times=1, delay=0.3):
    if left > right:
        my_rate = -rate
    else:
        my_rate = rate
        
    for i in xrange(times):
        ramp_servo_reliably(servo, left, my_rate)
        time.sleep(delay)
        ramp_servo_reliably(servo, right, -my_rate)
        time.sleep(delay)
    ramp_servo(servo, center, delay)

def wag_tail(times=5, delay=0.2):
    wag(TAIL, TAIL_LEFT, TAIL_RIGHT, TAIL_CENTER, times, delay)

def wag_head(times=5, delay=0.25):
    wag(HEAD, HEAD_LEFT, HEAD_RIGHT, HEAD_CENTER, times, delay)

def nod(times=3, delay=0.3):
    wag(NECK, NECK_DOWN, NECK_UP, NECK_UP, times, delay)

def open_mouth():
    set_servo_reliably(JAW, JAW_OPEN)

def close_mouth(with_ball=True):
    if with_ball:
        set_servo_reliably(JAW, JAW_CLOSED_FULL)
    else:
        set_servo_reliably(JAW, JAW_CLOSED_EMPTY)

def raise_head():
    ramp_servo(NECK, NECK_UP, 2)

def lower_head():
    ramp_servo(NECK, NECK_DOWN, -2)

def lower_legs():
    ramp_legs(LEGS_DOWN, -3, 0.02)

def raise_legs():
    ramp_legs(LEGS_UP, 3, 0.02)

def init_servos():
    set_legs(LEGS_UP)
    set_servo(NECK, NECK_UP)
    close_mouth()
    set_servo(HEAD, HEAD_CENTER)
    set_servo(TAIL, TAIL_CENTER)

def drop_ball():
    set_servo(HEAD, HEAD_CENTER)
    lower_head()
    lower_legs()
    open_mouth()
    raise_legs()
    close_mouth(with_ball=False)
    raise_head()
    set_servo(JAW,0) # relax jaw to prevent that servo overheating
    # wag_tail()

def grab_ball():
    set_servo(HEAD, HEAD_CENTER)
    open_mouth()
    lower_head()
    lower_legs()
    close_mouth()
    raise_legs()
    raise_head()
    set_servo(JAW,0) # relax jaw to prevent servo overheating
    wag_tail()

def test_servos(iterations=20):
    init_servos()
    for j in xrange(iterations):
        print "iteration {0}.".format(j)
        drop_ball()
        time.sleep(1)
        grab_ball()