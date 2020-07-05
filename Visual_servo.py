import vrep
import cv2
import numpy as np
import time
print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
if clientID!=-1:
    print ('Connected to remote API server')
    errorCode,cam_handle=vrep.simxGetObjectHandle(clientID,'cam1',vrep.simx_opmode_oneshot_wait)
    print ('Getting first image')
    time.sleep(0.05)
    errorCode, resolution, image=vrep.simxGetVisionSensorImage(clientID,cam_handle,0,vrep.simx_opmode_streaming_split+4000)
    time.sleep(0.05)
    
    while (vrep.simxGetConnectionId(clientID) != -1):
         errorCode, resolution, image=vrep.simxGetVisionSensorImage(clientID,cam_handle,0,vrep.simx_opmode_buffer)
         if errorCode == vrep.simx_return_ok:
             print('image OK!!!')
             ##im = np.array(image, dtype =np.uint8)
             im = np.array(image, dtype =np.uint8)
             im.resize([resolution [0], resolution[1], 3]) ##im.resize([resolution [1], resolution[0], 3])
             ##im = cv2.flip(im,1)
             rows, cols, _ = im.shape
             x_medium = int(cols / 2)
             center = int(cols / 2)
             position = 90 # degrees

             
             #BGR to HSV image
             #il = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
             hsv_frame= cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
             low_red = np.array([161, 155, 84])
             high_red = np.array([179, 255, 255])
             red_mask = cv2.inRange(hsv_frame, low_red, high_red)
             contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
             contours = sorted(contours, key=lambda x:cv2.contourArea(x), reverse=True)
    
             for cnt in contours:
                 x, y, w, h = cv2.boundingRect(cnt)
                 
        
        
                 x_medium = int((x + x + w) / 2)
                 
                 break
             frame = image
             cv2.line(im, (x_medium, 0), (x_medium, 480), (0, 255, 0), 2)
    
             #cv2.imshow("Frame", frame)
             cv2.imshow('im',hsv_frame )
             
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
    
                
 