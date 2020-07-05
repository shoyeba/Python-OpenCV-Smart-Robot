import vrep
import numpy as np
import matplotlib.pyplot as mlp
import time
#The required libraries 
print ('Program started')
vrep.simxFinish(-1)
 # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) 
# Connect to V-REP with the internal server
if clientID!=-1:
    print ('Connected to remote API server')

    #  retrieve data in a blocking fashion (i.e. a service call):
    res,objs=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_blocking)
    if res==vrep.simx_return_ok:
        print ('Number of objects in the scene: ',len(objs))
    else:
        print ('Remote API function call returned with error code: ',res)

errorCode,cam_handle=vrep.simxGetObjectHandle(clientID,'cam1',vrep.simx_opmode_oneshot_wait)
time.sleep(0.05)
errorCode, resolution, image=vrep.simxGetVisionSensorImage(clientID,cam_handle,0,vrep.simx_opmode_streaming)
 ## stream data from v-rep
time.sleep(0.05)
errorCode, resolution, image=vrep.simxGetVisionSensorImage(clientID,cam_handle,0,vrep.simx_opmode_buffer)
## buffer the stream data 
im = np.array(image, dtype =np.uint8)
##from the buffered data integer image array
im.resize([resolution [0], resolution[1], 3])
mlp.imshow(im,origin = 'lower')
##finally image is displayed
type(im)
