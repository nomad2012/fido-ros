#!/usr/bin/env python

# ball tracking using ROS and OpenCV

import roslib
roslib.load_manifest('fido')
import sys
import rospy
import cv
import std_msgs.msg      # import String
import sensor_msgs.msg   # import Image
from cv_bridge import CvBridge, CvBridgeError
import fido
import numpy as np
import datetime

def GetThresholdedImage(img):
  #returns thresholded image of the blue bottle
  imgHSV = cv.CreateImage(cv.GetSize(img), 8, 3)
  #converts a BGR image to HSV
  cv.CvtColor(img, imgHSV, cv.CV_BGR2HSV)
  imgThreshed = cv.CreateImage(cv.GetSize(img), 8, 1)
  #InRangeS takes source, lowerbound color, upperbound color and destination
  #It converts the pixel values lying within the range to 255 and stores it in
  #the destination
  cv.InRangeS(imgHSV, (30,100,90), (50, 130, 220), imgThreshed)
  return imgThreshed

posX = 0
posY = 0

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
    frame = bridge.imgmsg_to_cv(data, "bgr8")
  except CvBridgeError, e:
    print e

  # cv.Smooth(frame, frame, cv.CV_BLUR, 3)
  cv.Smooth(frame, frame, cv.CV_GAUSSIAN, 9, 9)
    
  imgThresh = GetThresholdedImage(frame)
  mat = cv.GetMat(imgThresh)
  moments = cv.Moments(mat, 0) 
  area = cv.GetCentralMoment(moments, 0, 0)
  moment10 = cv.GetSpatialMoment(moments, 1, 0)
  moment01 = cv.GetSpatialMoment(moments, 0, 1)
		
  lastX = posX
  lastY = posY

  if area > 20000: 
			
    # Calculate the coordinate postition of the centroid
    posX = int(moment10 / area)
    posY = int(moment01 / area)

    print str(datetime.datetime.now()), " frame: ", frame_number, ' x: ' + str(posX) + ' y: ' + str(posY) + ' area: ' + str(area) + ' head_x: ' + str(head_x) + ' neck_y: ' + str(neck_y) + ' jaw_pos: ' + str(jaw_pos)
    cv.ShowImage("orig_image", frame)

    # track the movement of the blob
    if(lastX > 0 and lastY > 0 and posX > 0 and posY > 0):
      if posX < CENTER_X - 10:
        error_x = (posX - CENTER_X) / MAX_X * (HEAD_RIGHT - HEAD_LEFT)
        desired_x = int(error_x) / 4 + head_x
        head_x = desired_x
        if head_x < HEAD_LEFT:
          head_x = HEAD_LEFT
        fido.set_servo(fido.HEAD, head_x)
      elif posX > CENTER_X + 10:
        new_x = (posX - CENTER_X) / MAX_X * (HEAD_RIGHT - HEAD_LEFT)
        head_x = int(new_x) / 4 + head_x
        if head_x > HEAD_RIGHT:
          head_x = HEAD_RIGHT
        fido.set_servo(fido.HEAD, head_x)

      if posY < CENTER_Y - 10:
        new_y = (posY - CENTER_Y) / MAX_Y * (NECK_UP - NECK_DOWN)
        neck_y = neck_y - (int(new_y) / 8)
        if neck_y > NECK_UP:
          neck_y = NECK_UP
        fido.set_servo(fido.NECK, neck_y)
      elif posY > CENTER_Y + 10:
        new_y = (posY - CENTER_Y) / MAX_Y * (NECK_UP - NECK_DOWN)
        neck_y = neck_y - (int(new_y) / 8)
        if neck_y < NECK_DOWN:
          neck_y = NECK_DOWN
        fido.set_servo(fido.NECK, neck_y)

      jaw_pos = (float(area) - 60000.0) / 1000000.0 * (fido.JAW_OPEN - fido.JAW_CLOSED_EMPTY) + fido.JAW_CLOSED_EMPTY
      jaw_pos = max(min(jaw_pos, fido.JAW_OPEN), fido.JAW_CLOSED_EMPTY)
      fido.set_servo(fido.JAW, int(jaw_pos))
  cv.WaitKey(3)

      
def main():
  global bridge
  global head_x, neck_y, jaw_pos

  print "balltrack initializing CvBridge"
  bridge = CvBridge()
  image_sub = rospy.Subscriber("/v4l/camera/image_raw", sensor_msgs.msg.Image, image_callback)

  fido.init_servos()
  fido.set_servo(fido.NECK, NECK_DOWN)
  head_x = fido.get_servo_position(fido.HEAD)
  neck_y = fido.get_servo_position(fido.NECK)
  jaw_pos = fido.get_servo_position(fido.JAW)
  print "balltrack opening window"
  cv.NamedWindow("orig_image", 1)
  cv.MoveWindow("orig_image", 0, 0)
  cv.NamedWindow("thresholded", 1)
  cv.MoveWindow("thresholded", 700, 0)

  print "balltrack init_node"
  rospy.init_node('balltrack', anonymous=True)
  try:
    print "balltrack spinning"
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"


if __name__ == "__main__":
  main()
