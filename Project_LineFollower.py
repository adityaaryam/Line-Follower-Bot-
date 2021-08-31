import cv2 as cv    
import numpy as np
import serial               # Used for serial communication with the Arduino
import time

# Using try except so that the code does not give an error while testing the iamge processing part.
# It will throw and error if the Arduino is not connected. Try-Except helps avoid this scenario.
try:
    arduino = serial.Serial(port='COM3', baudrate=115200, timeout=.005)
    arduino.close()
    arduino.open()
    f=1
except:
    f=0

kernel=np.ones((5,5))
def preprocess(image):
    imggray=cv.cvtColor(img,cv.COLOR_BGR2GRAY)
    imgblur=cv.GaussianBlur(imggray,(5,5),1)
    imgthreshed1=cv.threshold(imgblur,40,255,cv.THRESH_BINARY_INV)[1]    # Gray-Scaling, Blurring and thresholding helps in 
    return imgthreshed1                                                  # bettercontour detection
    

def compute(img):
    contours,h=cv.findContours(img,cv.RETR_EXTERNAL,cv.CHAIN_APPROX_NONE)

    # Following line of code gives the largest contour in the pre-processed frames
    # This is done so that only the path of the bot is detected and the noise would be refrained
    biggestcnt=max(contours,key=cv.contourArea)
    cv.drawContours(imgcontour,biggestcnt,-1,(255,0,255),3)

    # Method of moments to find the centroid of the contour and the drawing an indicating point
    mo=cv.moments(biggestcnt)
    cx=int(mo["m10"]/mo["m00"])
    cy=int(mo["m01"]/mo["m00"])
    cv.circle(imgcontour,(cx,cy),5,(255,0,0),cv.FILLED)

    # Calculating the error based on segmentation of frames into 3 parts
    # Error is 0 when indicating point is between 190 and 290
    # Error is negative when the indicating point is beyond 290
    # Error is positive when the indicating point is below 190
    error=0
    if(cy>290):
        print("left")               # Printing left, on track and right has nothing to do with the logic.
        error=290-cy                # It is only for the reference while testing the image processing. 
    if( cy>=190  and cy<=290):
        print("on track")
    if(cy<190):
        print("right")
        error=190-cy
    
    print(error)                                            # Printing error only for reference

    if(f==1):
        time.sleep(0.005)
        arduino.write(bytes(str(error), 'utf-8'))           # Sending Error to the Serial Port which is being received by the Arduino
    
    
# vid=cv.VideoCapture('http://192.168.29.13:4747/video')    # Using Droid Cam to get live feed from the camera
vid=cv.VideoCapture('http://192.168.29.46:4747/video')


while True:
    success,img=vid.read()
    imgcontour=img.copy()
    imgpreproc=preprocess(img)                                              # This function pre-proccesses the image 
    compute(imgpreproc)                                                     # Detects the path and computes the error i.e. deviation
    cv.line(imgcontour, (0,190), (640,190), (0, 255, 0), thickness=5)
    cv.line(imgcontour, (0,290), (640,290), (0, 255, 0), thickness=5)       # Two Lines for Visual Reference   
    cv.imshow("Path",imgcontour)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

vid.release()
cv.destroyAllWindows()