
"""
Computer Vision Based Robotic Guidance System V-rep Code
Created on Sat Dec  7 17:58:53 2019

@author: ahammed Shoyeb
"""

#Libraries
import vrep
import cv2
import numpy as np
import time
import imutils
print ('Program started')
def find_marker(image):
	# convert the image to grayscale, blur it, and detect edges
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	gray = cv2.GaussianBlur(gray, (5, 5), 0)
	edged = cv2.Canny(gray, 35, 125)

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

vrep.simxFinish(-1) # just in case, close all opened connections
#clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
if clientID!=-1:
    print ('Connected to remote API server')
    #camera control Motor control*****************

    errorCode,cam_handle=vrep.simxGetObjectHandle(clientID,'cam1',vrep.simx_opmode_oneshot_wait)
    errorCode,left_motor_handle=vrep.simxGetObjectHandle(clientID,'rjlf',vrep.simx_opmode_oneshot_wait)
    errorCode,right_motor_handle=vrep.simxGetObjectHandle(clientID,'rjrf',vrep.simx_opmode_oneshot_wait)
    errorCode,left_front_motor_handle=vrep.simxGetObjectHandle(clientID,'rjl',vrep.simx_opmode_oneshot_wait)
    errorCode,right_front_motor_handle=vrep.simxGetObjectHandle(clientID,'rjr',vrep.simx_opmode_oneshot_wait)
    #*********************************************
    print ('Getting first image')
    time.sleep(0.05)
    errorCode, resolution, image=vrep.simxGetVisionSensorImage(clientID,cam_handle,0,vrep.simx_opmode_streaming_split+4000)
    
        
    while (vrep.simxGetConnectionId(clientID) != -1):
         errorCode, resolution, image=vrep.simxGetVisionSensorImage(clientID,cam_handle,0,vrep.simx_opmode_buffer)
         if errorCode == vrep.simx_return_ok:
             print('image OK!!!')
             
             im = np.array(image, dtype =np.uint8)
             im.resize([resolution [0], resolution[1], 3]) 
             
             time.sleep(0.09)
             rows, cols, _ = im.shape
             x_medium = int(cols / 2)
             center = int(cols / 2)
             position = 90 # degrees

             hsv_frame= cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
             low_red = np.array([161, 155, 84])
             high_red = np.array([179, 255, 255])
             red_mask = cv2.inRange(hsv_frame, low_red, high_red)
             marker = find_marker(im)
             #marker = find_marker(red_mask)
             inches = distance_to_camera(KNOWN_WIDTH, focalLength, marker[1][0])
             box = cv2.cv.BoxPoints(marker) if imutils.is_cv2() else cv2.boxPoints(marker)
             box = np.int0(box)
##****************************************************************

             contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
             contours = sorted(contours, key=lambda x:cv2.contourArea(x), reverse=True)
             cv2.drawContours(im, [box], -1, (0, 255, 0), 2)
             cv2.putText(im, "%.2fft" % (inches / 12),(im.shape[1] - 200, im.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX,2.0, (0, 255, 0), 3)
##****************************************************************
             location = int (inches/12)
             for cnt in contours:
                 (x, y, w, h) = cv2.boundingRect(cnt)
                 x_medium = int((x + x + w) / 2)
                 break
             
                
#Folloing Line of code detects the Red blob location from the centre pixel
#If the blob is located less than -30 pixel and distance greater than 1 feet away 
#it it ldiplay 'left' and turn on right side motor forward    
                 
             if x_medium < center -30  and location >1:
                 print('left')
                 

                 errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,0.3, vrep.simx_opmode_streaming)
                 errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,0.1, vrep.simx_opmode_streaming)
                 errorCode=vrep.simxSetJointTargetVelocity(clientID,left_front_motor_handle,0.3, vrep.simx_opmode_streaming)
                 errorCode=vrep.simxSetJointTargetVelocity(clientID,right_front_motor_handle,0.1, vrep.simx_opmode_streaming)
                 
#Folloing Line of code detects the Red blob location from the centre pixel
#If the blob is located greater than -30 pixel and distance greater than 1 feet away 
#it it ldiplay 'right' and turn on Left side motor forward       
        
             elif x_medium > center + 30 and location >1:
                 print('right')

                 errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,0.1, vrep.simx_opmode_streaming)
                 errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,0.3, vrep.simx_opmode_streaming)
                 errorCode=vrep.simxSetJointTargetVelocity(clientID,left_front_motor_handle,0.1, vrep.simx_opmode_streaming)
                 errorCode=vrep.simxSetJointTargetVelocity(clientID,right_front_motor_handle,0.3, vrep.simx_opmode_streaming)  
                 
#Folloing Line of code detects the Red blob location from the centre pixel
#If the blob is located distance less than 1 feet away 
#it it ldiplay 'reverse' and turnon both side motor forward       
                 
             elif location  < 2 :
                 print('reverse_now')


                 errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,-0.9, vrep.simx_opmode_streaming)
                 errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,-0.9, vrep.simx_opmode_streaming)
                 errorCode=vrep.simxSetJointTargetVelocity(clientID,left_front_motor_handle,-0.9, vrep.simx_opmode_streaming)
                 errorCode=vrep.simxSetJointTargetVelocity(clientID,right_front_motor_handle,-0.9, vrep.simx_opmode_streaming)
                 
#Folloing Line of code detects the Red blob location from the centre pixel
#If the blob is located in  -30 to +30 from centre pixel and distance greater than 1 feet away 
#it it ldiplay 'left' and turnon both side motor forward 
                 
             else:
                 print('centre')


                 errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,0.3, vrep.simx_opmode_streaming)
                 errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,0.3, vrep.simx_opmode_streaming)
                 errorCode=vrep.simxSetJointTargetVelocity(clientID,left_front_motor_handle,0.3, vrep.simx_opmode_streaming)
                 errorCode=vrep.simxSetJointTargetVelocity(clientID,right_front_motor_handle,0.3, vrep.simx_opmode_streaming)
                

    
             frame = image
             cv2.line(im, (x_medium, 0), (x_medium, 256), (0, 255, 0), 2)

             cv2.imshow('HSV_FRAME',im )##
             
             
             #command for proram termination             
             if cv2.waitKey(1) & 0xFF == ord('q'):
                 break
             elif errorCode == vrep.simx_return_novalue_flag:
                 print ("no image yet")
                 pass
             else:
                 print ('error/')
            
             
else:
    print("Failed to connect to remote API Server")
    vrep.simxFinish(clientID)
    

cv2.destroyAllWindows()    
    
                
 