import cv2
from video import create_capture
import fido
import numpy as np
import time
from datetime import datetime

def GetThresholdedImage(img):
  #returns thresholded image of the blue bottle

  #converts a BGR image to HSV
  imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
  #InRangeS takes source, lowerbound color, upperbound color and destination
  #It converts the pixel values lying within the range to 255 and stores it in
  #the destination
  imgThreshed = cv2.inRange(imgHSV, (30,70,128), (43, 144, 220))
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
NECK_START = (NECK_DOWN + NECK_CENTER) / 2


def main():
  color_tracker_window = "output"
  thresh_window = "thresh"
  blob_window = "blob"
  cam = create_capture(-1)
  #cv2.namedWindow(color_tracker_window, 1)
  #cv2.moveWindow(color_tracker_window, 0, 0)
  #cv2.namedWindow(thresh_window, 1)
  #cv2.moveWindow(thresh_window, 700, 0)
  #cv2.namedWindow(blob_window, 1)
  #cv2.moveWindow(blob_window, 700, 500)
  imgScrible = None
  storage = None
  global posX
  global posY

  fido.init_servos()
  time.sleep(0.25)
  fido.set_servo(fido.NECK, NECK_START)
  head_x = fido.get_servo_position(fido.HEAD)
  neck_y = fido.get_servo_position(fido.NECK)
  jaw_pos = fido.get_servo_position(fido.JAW)
   
  #frame = cv.QueryFrame(capture)
  #imgThresh = GetThresholdedImage(frame)

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
    blurred = cv2.GaussianBlur(frame, (9, 9), 0)
    imgThresh = GetThresholdedImage(blurred)
    #cv2.imshow(thresh_window, imgThresh)
    #cv2.waitKey(1)
    
    #Calculating the moments
    #contours = None
    contours, hierarchy = cv2.findContours(imgThresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    area = 0
    if contours:
      for c in contours:
        moments = cv2.moments(c)
        a = moments['m00']
        if a > area:
          area = a
          moment10 = moments['m10']
          moment01 = moments['m01']

    
    #lastX and lastY stores the previous positions
    lastX = posX
    lastY = posY

    #Finding a big enough blob
    if area > 100: 
			
      #Calculating the coordinate postition of the centroid
      posX = int(moment10 / area)
      posY = int(moment01 / area)

      last_known_x = posX
      last_known_y = posY

    elif last_known_x is not None:
      if head_x > HEAD_LEFT and head_x < HEAD_RIGHT and neck_y > NECK_DOWN and neck_y < NECK_UP:
        posX = last_known_x
        posY = last_known_y
      else:
        fido.ramp_servo(fido.HEAD, HEAD_CENTER, -3 if head_x > HEAD_CENTER else 3)
        fido.ramp_servo(fido.NECK, NECK_START, -3 if neck_y > NECK_CENTER else 3)
        posX = CENTER_X
        posY = CENTER_Y
        last_known_x = None
        last_known_y = None

    if True:

      print str(datetime.now()), ' x: ' + str(posX) + ' y: ' + str(posY) + ' area: ' + str(area) + ' head_x: ' + str(head_x) + ' neck_y: ' + str(neck_y) + ' jaw_pos: ' + str(jaw_pos)
      #drawing lines to track the movement of the blob
      if(lastX > 0 and lastY > 0 and posX > 0 and posY > 0):
        #cv.Circle( imgThresh, (posX, posY), maxRadius, cv.Scalar(0,0,255), 3, 8, 0 );        
        #cv.Line(imgScrible, (posX, posY), (lastX, lastY), cv.Scalar(0, 0, 255), 5)
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

        jaw_pos =int((float(area) - 1000.0) / 25000.0 * (fido.JAW_OPEN - fido.JAW_CLOSED_EMPTY) + fido.JAW_CLOSED_EMPTY)
        jaw_pos = max(min(jaw_pos, fido.JAW_OPEN), fido.JAW_CLOSED_EMPTY)
        fido.set_servo(fido.JAW, jaw_pos)
      #Adds the three layers and stores it in the frame
      #frame -> it has the camera stream
      #imgScrible -> it has the line tracking the movement of the blob
      #cv.Add(frame, imgScrible, frame)
    else:
      print "area = {}".format(area)

    #cv2.imshow(blob_window, imgThresh)
    end_time = time.time()
    print "elapsed time = {}".format(end_time - start_time)
    #c = cv2.waitKey(3)
    #if(c!=-1):
    #  break
  print "exiting"
  #cv2.destroyAllWindows()


if __name__ == "__main__":
  main()
