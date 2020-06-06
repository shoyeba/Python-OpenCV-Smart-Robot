#https://download.01.org/opencv/2019/open_model_zoo/R2/20190628_180000_models_bin/vehicle-detection-adas-0002/FP16/
import cv2
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(18,GPIO.OUT)
GPIO.output(18, GPIO.HIGH)

# Load the model.
net = cv2.dnn.readNet('vehicle-detection-adas-0002.xml',
                     'vehicle-detection-adas-0002.bin')
#net = cv2.dnn.readNet('face-detection-adas-0001.xml',
#                     'face-detection-adas-0001.bin')

# Specify target device.
net.setPreferableTarget(cv2.dnn.DNN_TARGET_MYRIAD)
#cap = cv2.VideoCapture(0)
cap = cv2.VideoCapture('/home/pi/ov/rvd.mp4')


while(cap.isOpened()):
  # Capture frame-by-frame
  ret, frame = cap.read()



  if ret == True:
#      blob = cv2.dnn.blobFromImage(frame, size=(672, 384), ddepth=cv2.CV_8U)
#      blob = cv2.dnn.blobFromImage(frame, size=(1600, 900), ddepth=cv2.CV_8U)
      blob = cv2.dnn.blobFromImage(frame, size=(1600, 900), ddepth=cv2.CV_8U)

#      net.setInput(blob)
      net.setInput(blob)
      out = net.forward()
  for detection in out.reshape(-1, 7):
    confidence = float(detection[2])
    p = int(confidence * 10) 
    
    xmin = int(detection[3] * frame.shape[1])
    ymin = int(detection[4] * frame.shape[0])
    xmax = int(detection[5] * frame.shape[1])
    ymax = int(detection[6] * frame.shape[0])

    if p>2:
                    
            
            
        image = cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), color=(0, 255, 30)) 
        cv2.putText(frame,"Vehicle Detected",(10,150),cv2.FONT_HERSHEY_TRIPLEX, 1.2,(0,255,255),2)
        cv2.putText(frame,"Latitude-52.4081812, Longitude--1.510477",(10,50),cv2.FONT_HERSHEY_TRIPLEX, .7,(150,155,255),2)
        cv2.putText(frame,"OpenVINO CAFFE",(10,100),cv2.FONT_HERSHEY_TRIPLEX, 1.2,(0,255,0),1)
        cv2.putText(frame,"Lockdown Traffic",(10,400),cv2.FONT_HERSHEY_TRIPLEX, 1.2,(0,0,255),1)
        cv2.putText(frame,"Pi4CAM",(10,450),cv2.FONT_HERSHEY_TRIPLEX, 1.2,(255,0,0),1)
        cv2.putText(frame,">>>>>",(10,200),cv2.FONT_HERSHEY_TRIPLEX, 2,(255,0,255),1)
        print(confidence)

  cv2.imshow("Frame", frame)
    
    
  k = cv2.waitKey(1)
    
  if k == 27:
      break
  elif k == ord('s'):
      
      # wait for 's' key to save and exit
      cap.release()
          
      cv2.destroyAllWindows()


