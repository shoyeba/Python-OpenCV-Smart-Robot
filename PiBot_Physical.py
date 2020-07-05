##!/usr/bin/env python3
## -*- coding: utf-8 -*-
#"""
#Created on Sat Feb  1 21:41:07 2020
#
#@author: pi
#"""
#
#from gpiozero import Robot
#
#
#from time import sleep
#robot = Robot(left = (18,22), right = (17, 23))
#while True:
#	robot.forward()
#	sleep(3)
#	robot.stop()
#	robot.right()
#	sleep(1)
#	robot.stop()
import cv2
import numpy as np

import RPi.GPIO as GPIO

from time import sleep

import imutils

def find_marker(image):
	# convert the image to grayscale, blur it, and detect edges
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	blur = cv2.GaussianBlur(gray, (5, 5), 0)
	edged = cv2.Canny(blur, 35, 125)

	# find the contours in the edged image and keep the largest one;
	# we'll assume that this is our piece of paper in the image
	cnts = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	c = max(cnts, key = cv2.contourArea)

	# compute the bounding box of the of the paper region and return it
	return cv2.minAreaRect(c)
def distance_to_camera(knownWidth, focalLength, perWidth):
	# compute and return the distance from the maker to the camera
	return (knownWidth * focalLength) / perWidth
KNOWN_DISTANCE = 24.0

# initialize the known object width, which in this case, the piece of
# paper is 12 inches wide
KNOWN_WIDTH = 11.0
image = cv2.imread("images/2ft.png")
marker = find_marker(image)
focalLength = (marker[1][0] * KNOWN_DISTANCE) / KNOWN_WIDTH

cap = cv2.VideoCapture(0)
#cap = cv2.VideoCapture(0)
cap.set(3, 480)
cap.set(4, 320)
#cap.set(4, 480)

 
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(17,GPIO.OUT)
GPIO.setup(18,GPIO.OUT)
GPIO.setup(22,GPIO.OUT)
GPIO.setup(23,GPIO.OUT)
r = GPIO.PWM(17, 0.9)
l = GPIO.PWM(18, 0.9)## GPIO.PWM(channel, frequency)

rr = GPIO.PWM(22, 0.9)
rl = GPIO.PWM(23, 0.9)

 
# Check if camera opened successfully
if (cap.isOpened()== False): 
  print("Error opening video stream or file")
  r.start(0)#        GPIO.output(17,GPIO.HIGH)
  l.start(0)#        GPIO.output(18,GPIO.HIGH)
  rr.stop()#        GPIO.output(22,GPIO.LOW)
  rl.stop()#        GPIO.output(23,GPIO.LOW)
 
# Read until video is completed


## Distance Manual******************************************************************
while(cap.isOpened()):
  # Capture frame-by-frame
  ret, frame = cap.read()


  _, frame = cap.read()
  rows, cols, _ = frame.shape

  x_medium = int(cols / 2)
  center = int(cols / 2)
  position = 90 # degrees
  if ret == True:
      
      print( "i m opened")
      hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
      #cv2.imshow("hsv Image", hsv_frame)
      low_blue = np.array([110,50,50])  ## BGR
      high_blue = np.array([130,255,255]) ##BGR
      blue_mask = cv2.inRange(hsv_frame, low_blue, high_blue)

      res = cv2.bitwise_and(hsv_frame,hsv_frame, mask= blue_mask)

      marker = find_marker(res)

      inches = distance_to_camera(KNOWN_WIDTH, focalLength, marker[1][0])
      box = cv2.cv.BoxPoints(marker) if imutils.is_cv2() else cv2.boxPoints(marker)
      box = np.int0(box)
      contours, _ = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
      contours = sorted(contours, key=lambda x:cv2.contourArea(x), reverse=True)
      cv2.drawContours(frame, [box], -1, (0, 255, 0), 2)
#      cv2.putText(frame, "%.2fcm" % (inches / 12),(frame.shape[1] - 200, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX,2.0, (0, 255, 0), 3)
            
      cv2.putText(frame, "%.2fcm" % (inches / 5),(frame.shape[1] - 260, frame.shape[0] -40), cv2.FONT_HERSHEY_SIMPLEX,2.0, (0, 255, 0), 3)

      sleep(0.2)
      location = int (inches/5)
      #print (location)
      for cnt in contours:
        (x, y, w, h) = cv2.boundingRect(cnt)
        cv2.rectangle (res, (x,y), (x+w, y+h), (0,255,0),2)
        
        x_medium = int((x + x + w) / 2)
        break
    
      if x_medium < center -30 and location >1:
          print('left',location)

          r.stop()#        GPIO.output(17,GPIO.LOW)
          l.start(30)##Duty cycle#       GPIO.output(18,GPIO.HIGH)
        
          rr.stop()#        GPIO.output(22,GPIO.LOW)
          rl.stop() #        GPIO.output(23,GPIO.LOW)      
            
        
      elif x_medium > center + 30 and location >1:
          print('right',location)

          r.start(30)#        GPIO.output(17,GPIO.HIGH)##Duty cycle (30)
          l.stop()#        GPIO.output(18,GPIO.LOW)
          rr.stop()#        GPIO.output(22,GPIO.LOW)
          rl.stop()#        GPIO.output(23,GPIO.LOW)
        
      elif location < 1 :
          print('reverse_now',location)




          r.start(20)#        GPIO.output(22,GPIO.HIGH)
          l.start(20)#        GPIO.output(23,GPIO.HIGH)
          rr.start(100)#        GPIO.output(17,GPIO.HIGH)
          rl.stop(100)#        GPIO.output(18,GPIO.HIGH)
        
      else:
          print('Centre',location)
        
          sleep(0.1)

          r.start(50)#        GPIO.output(17,GPIO.HIGH)
          l.start(50)#        GPIO.output(18,GPIO.HIGH)
          rr.stop()#        GPIO.output(22,GPIO.LOW)
          rl.stop()#        GPIO.output(23,GPIO.LOW)
        
        
    
      cv2.line(res, (x_medium, 0), (x_medium, 480), (0, 255, 0), 2)
      cv2.rectangle (res, (x,y), (x+w, y+h), (0,255,0),2)
    
      #cv2.imshow("Frame", frame)
      #cv2.imshow("Frame", res)
    
    
      k = cv2.waitKey(1)
    
      if k == 27:
          break
      elif k == ord('s'): # wait for 's' key to save and exit
          cap.release()
          GPIO.cleanup()
          cv2.destroyAllWindows()


