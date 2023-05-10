import RPi.GPIO as GPIO
import picamera
from time import sleep, time
from picamera import PiCamera
from picamera.array import PiRGBArray
import numpy as np
import cv2
import sys
import signal

#cleanup after closing
def termhandler(sig, frame):
    cv2.destroyAllWindows()
    rightmotor.stop()
    leftmotor.stop()
    finish_time = time()
    fps = counter / (finish_time - start_time)
    print("Frames per second = " + str(fps))

    sys.exit()
    
signal.signal(signal.SIGINT, termhandler)

#Setting up the GPIO pins
GPIO.setmode(GPIO.BOARD)
GPIO.setup(32, GPIO.OUT)
GPIO.setup(33, GPIO.OUT)
rightmotor = GPIO.PWM(32, 1000)
leftmotor = GPIO.PWM(33, 1000)
rightmotor.start(0)
leftmotor.start(0)
 
#setpoint to calculate error
setpoint = 208
error = 0
counter = 0
resolution = (416, 112) #(640, 368)
previous_error = 0

 
def pid(err, prev_err):

    kp = 0.01
    ki = 1
    kd = 10
    speed = 90
    
    p = err
    i = err + prev_err    
    d = err - prev_err
    
        
    pidout = (kp*p) + (ki*i) + (kd*d)

    left = speed + pidout
    right = speed - pidout
    
    if left > 100:
        left  = 100
        
    if left < 0:
        left  = 0
        
    if right > 100:
        right  = 100
        
    if right < 0:
        right = 0
        
    rightmotor.ChangeDutyCycle(right)
    leftmotor.ChangeDutyCycle(left)

   # print("Error:", err)
    #print("PID: ", pidout)
    #print("Right: ", right)
    #print("Left: ", left)
    

camera = PiCamera()

camera.resolution = resolution
#camera.rotation = 90
camera.framerate=60

#make a buffer
capture = PiRGBArray(camera, size=resolution)
#sleep(1)


start_time = time()

for frame in camera.capture_continuous(capture, format="bgr", use_video_port=True):
    
    #used to calculate frames per second
    counter += 1
    #load photo
    photo = frame.array
    
    #capture only the black line
    photoBW = cv2.inRange(photo, (0,0,0), (50,50,50))
    
    #3x3 matrix for erosion
    matrix = np.ones((3,3), np.uint8)
    
    #erode to remove noise from the contour
    photoBW = cv2.erode(photoBW, matrix, iterations=1)
    
    #dialate after noise has been removed
    photoBW = cv2.dilate(photoBW, matrix, iterations=1)
    
    #find the contour of the black line
    contour, a = cv2.findContours(photoBW.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    #draw contour on the captured photo
    #cv2.drawContours(photo, contour, -1, (0,0,255), 3)
    
    #draw a bounding rectangle in on the contour
    if len(contour) > 0:
        #get coordinates of the bouding rectangle
        x,y,w,h = cv2.boundingRect(contour[0])
        
        blk_cntr = round(x+w/2)
        
        #draw rectangle
        #cv2.rectangle(photo, (x,y), (x+w, y+h), (255,0,0), 3)
        
        #draw a line in the middle of the rectangle
        cv2.line(photo, (blk_cntr, 0), (blk_cntr, 368), (0,255,0), 3)
        
    #calculate error
    error = blk_cntr - setpoint
    pid(error, previous_error)
    previous_error = error
    #cv2.putText(photo, "error="+str(error), (200, 340), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,0,255))
    #show photo
    cv2.imshow("Original", photo)

    
    #clear buffer
    capture.truncate(0)
    
    if cv2.waitKey(1) == 113:
        cv2.destroyAllWindows()
        rightmotor.stop()
        leftmotor.stop()
        
        break
    
finish_time = time()
fps = counter/(finish_time - start_time)
print("Frames per second = " + str(fps))
        
    

    
    
