#!/usr/bin/env python

# ball tracking using ROS and OpenCV

import roslib
roslib.load_manifest('fido')
import sys
import rospy
import cv2
import std_msgs.msg      # import String
import sensor_msgs.msg   # import Image
from cv_bridge import CvBridge, CvBridgeError
import fido
import numpy as np
import datetime
#from guppy import hpy


posX = 0
posY = 0

MAX_X = 320.0
MAX_Y = 240.0

Px = 8
Py = 16

CENTER_X = MAX_X / 2.0
CENTER_Y = MAX_Y / 2.0

HEAD_CENTER = 106
HEAD_RIGHT = HEAD_CENTER + 40
HEAD_LEFT = HEAD_CENTER - 40

NECK_UP = 230
NECK_DOWN = 100
NECK_CENTER = (NECK_UP + NECK_DOWN) / 2

head_x = 127
neck_y = 127
jaw_pos = 127

bridge = None

frame_number = 0

def image_callback(data):
  global posX, posY
  global head_x, neck_y, jaw_pos
  global bridge
  global frame_number
  
  frame_number += 1

  try:
    imgThresh = bridge.imgmsg_to_cv(data, "8UC1")
  except CvBridgeError, e:
    print e

  #mat = cv.GetMat(imgThresh)
  #moments = cv.Moments(mat, 0) 
  #area = cv.GetCentralMoment(moments, 0, 0)
  #moment10 = cv.GetSpatialMoment(moments, 1, 0)
  #moment01 = cv.GetSpatialMoment(moments, 0, 1)

  contours, hierarchy = cv2.findContours(imgThresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
  moments = cv2.moments(contours)
  area = moments['m00']
  moment10 = moments['m10']
  moment01 = moments['m01']
  
  lastX = posX
  lastY = posY

  if area > 10000: 
			
    # Calculate the coordinate postition of the centroid
    posX = int(moment10 / area)
    posY = int(moment01 / area)
        
    print "{0} frame: {1} x: {2} y: {3} area: {4} head_x: {5} neck_y: {6} jaw_pos: {7}".format(
      datetime.datetime.now(), frame_number, posX, posY, area, head_x, neck_y, jaw_pos)

    #if frame_number % 100 == 0:
    #  h = hpy()
    #  print h.heap()

    # track the movement of the blob
    if(lastX > 0 and lastY > 0 and posX > 0 and posY > 0):
      if posX < CENTER_X - 10:
        error_x = (posX - CENTER_X) / MAX_X * (HEAD_RIGHT - HEAD_LEFT)
        desired_x = int(error_x) / Px + head_x
        head_x = desired_x
        if head_x < HEAD_LEFT:
          head_x = HEAD_LEFT
        fido.set_servo(fido.HEAD, head_x)
      elif posX > CENTER_X + 10:
        new_x = (posX - CENTER_X) / MAX_X * (HEAD_RIGHT - HEAD_LEFT)
        head_x = int(new_x) / Px + head_x
        if head_x > HEAD_RIGHT:
          head_x = HEAD_RIGHT
        fido.set_servo(fido.HEAD, head_x)

      if posY < CENTER_Y - 10:
        new_y = (posY - CENTER_Y) / MAX_Y * (NECK_UP - NECK_DOWN)
        neck_y = neck_y - (int(new_y) / Py)
        if neck_y > NECK_UP:
          neck_y = NECK_UP
        fido.set_servo(fido.NECK, neck_y)
      elif posY > CENTER_Y + 10:
        new_y = (posY - CENTER_Y) / MAX_Y * (NECK_UP - NECK_DOWN)
        neck_y = neck_y - (int(new_y) / Py)
        if neck_y < NECK_DOWN:
          neck_y = NECK_DOWN
        fido.set_servo(fido.NECK, neck_y)

      jaw_pos = (float(area) - 60000.0) / 1000000.0 * (fido.JAW_OPEN - fido.JAW_CLOSED_EMPTY) + fido.JAW_CLOSED_EMPTY
      jaw_pos = max(min(jaw_pos, fido.JAW_OPEN), fido.JAW_CLOSED_EMPTY)
      fido.set_servo(fido.JAW, int(jaw_pos))
  #cv.WaitKey(3)
  # outgoing = bridge.cv_to_imgmsg(imgThresh, "8UC1")

      
def main():
  global bridge
  global head_x, neck_y, jaw_pos

  print "balltrack2 initializing CvBridge"
  bridge = CvBridge()
  image_sub = rospy.Subscriber("/fido/balldetect_image", sensor_msgs.msg.Image, image_callback)

  fido.init_servos()
  fido.set_servo(fido.NECK, NECK_DOWN)
  head_x = fido.get_servo_position(fido.HEAD)
  neck_y = fido.get_servo_position(fido.NECK)
  jaw_pos = fido.get_servo_position(fido.JAW)

  print "balltrack2 init_node"
  rospy.init_node('balltrack', anonymous=True)
  try:
    print "balltrack2 spinning"
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting2 down"


if __name__ == "__main__":
  main()
