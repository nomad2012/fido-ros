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
image_pub = None

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
  try:
    image_pub.publish(bridge.cv_to_imgmsg(imgThresh, "8UC1"))
  except CvBridgeError, e:
    print e

    
def main():
  global bridge, image_pub
  global head_x, neck_y, jaw_pos

  print "balldetect init_node"
  rospy.init_node('balldetect', anonymous=True)

  print "balldetect initializing CvBridge"
  bridge = CvBridge()
  image_pub = rospy.Publisher("/fido/balldetect_image", sensor_msgs.msg.Image)
  image_sub = rospy.Subscriber("/v4l/camera/image_raw", sensor_msgs.msg.Image, image_callback)
  
  try:
    print "balldetect spinning"
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"


if __name__ == "__main__":
  main()
