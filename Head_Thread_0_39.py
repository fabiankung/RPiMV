# -*- coding: utf-8 -*-
"""
Codes to:
1. Enable multi-threading.
2. Open a serial communication thread using pySerial to interface with
   external Robot Controller (RC).
3. Open raspiberry pi camera thread - picamera using native interface in a thread.
   Default camera resolution is 320x240 pixels. User can change to 640x480
   or 160x120 (Note: we can also rescale the image and print it
   out at larger dimension).
4. Supports two image processing algorithms:
   a) Run face detection algorithim using Haar cascades method in OpenCV2.
   b) Sparse optical flow method using Shi-Tomasi feature tracking method in OpenCV2.
5. Turn on GPIO24 pin to drive external LED.
6. Send instruction to turn RC to control elevation and azimuth motors so
   that the pi-camera can track the face.
7. To quit, press the key 'q' on the keyboard. 

Author       : Fabian Kung
Last modified: 12 May 2021
"""

import threading
import serial
import cv2
import numpy as np
from picamera import PiCamera
from picamera.array import PiRGBArray
import time
from gpiozero import PWMLED

_FRAME_WIDTH = 320  # In pixels.
_FRAME_HEIGHT = 240 # In pixels.
_FRAME_WIDTH_DIV2 = 160 # In pixels.
_FRAME_HEIGHT_DIV2 = 120 # In pixels.
_FRAME_RATE = 24    # In frames-per-second.

_LED_NORMAL_BRIGHTNESS = 0.1  # Normal brightness of LED, 10%.
_LED_MAX_BRIGHTNESS = 0.9

#Define serial communication class
class SerialCommThread(threading.Thread):
    ReadData = []
    def __init__(self, name):           # Overload constructor.
        threading.Thread.__init__(self) # Run original constructor. 
        self.name = name
        self.stopped = False
        self.TXcount = 0
        self.RC_TX_Busy = False
        self.utf8TXcommandstring = ""
    
    def run(self):                      # Overload run() method.
        while self.stopped == False:
            if self.RC_TX_Busy == False: # If no data string to send to Robot Controller (RC), poll RC status.
                #self.txcount = sport.write(bytes([71,98 ,48,48,48,10])) #"Gb000\n" Get robot controller
                                                                   # parameters in binary.                                                                   
                #readData = sport.read(8)                               # Read 8 bytes from RC.
                self.TXcount = sport.write(bytes([71,68 ,48,48,48,10])) #"GD000\n" Get robot controller
                                                                   # distance output.                                                                  
                ReadData = sport.read(5)                           # Read 5 bytes from RC.
                sport.reset_input_buffer()                         # Flush serial port      

                #if ReadData.__sizeof__() >= 22:                    # NOTE: 27 April 2021, the read() 
                                                                   # method of pySerial returns an array
                                                                   # of bytes, which has an overhead of 
                                                                   # 33 bytes. Even if read() timeout, we
                                                                   # still get 33 bytes. So if we expect
                                                                   # 5 bytes, we need to add the expected
                                                                   # bytes with the overhead.
                #    print(ReadData)
                print(ReadData)
                #print(ReadData[0])
                #print(ReadData.__sizeof__())              
            else:                                                # If there is data to send to RC.
                #print(self.utf8TXcommandstring)
                self.TXcount = sport.write(self.utf8TXcommandstring)    
                ReadData = sport.read(2)                         # Read 2 bytes from RC, "OK".
                sport.reset_input_buffer()                           # Flush serial port      
                self.RC_TX_Busy = False                          # Clear busy flag.
                
            time.sleep(0.02)                                     # 20 msec delay. So this thread will
                                                                 # Check for data string to be send to the RC
                                                                 # or get updates on the RC status 50 times
                                                                 # a second, which is good (earlier version
                                                                 # is 10 times/second).
        
    def stop(self):
        # Indicate that the thread should be stopped.
        self.stopped = True
        sport.close()                                           # Note: 8 May 2021. I noticed that if we close the
                                                                # COM port inside the thread, it may raise exception
                                                                # during the execution as other threads may still be
                                                                # using the COM port. Thus other methods in this thread
                                                                # needs to check this flag too.
        print("Port %s is closed" % sport.port)
        
    def sendtoRC(self, string):
        if self.RC_TX_Busy == False and self.stopped == False:
            self.RC_TX_Busy = True                              # Tell the run() thread there is data to send to RC.
            self.utf8TXcommandstring = string
            return True
        else:
            return False

# Define Raspberry Pi camera class
class CamThread:
    def __init__(self, resolution = (320,240), framerate = 24): # Constructor.
        # Initialize the pi camera parameters and class parameters.
        self.camera = PiCamera()
        self.camera.resolution = resolution
        self.camera.framerate = framerate
        self.rawCapture = PiRGBArray(self.camera, size=resolution)
        self.stream = self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True)
        self.img = None
        self.stopped = False

    def start(self):
        # Start the thread to read frames from the video stream.
        threading.Thread(target=self.update, args=()).start()
        return self

    def update(self):
        # Keep looing indefinitely until the thread is stopped.
        for f in self.stream:
            # Grab the frame from the stream and clear the stream in
            # preparation for the next frame.
            self.img = f.array
            self.rawCapture.truncate(0)

            # If thread stop flag is set, stop the thread and release
            # camera resources.
            if self.stopped:
                self.stream.close()
                self.rawCapture.close()
                self.camera.close()
                return
            
    def read(self):
        # Return the frame most recently read.
        return self.img
        #return cv2.cvtColor(self.img,cv2.COLOR_BGR2GRAY)

    def stop(self):
        # Indicate that the thread should be stopped.
        self.stopped = True

# Initialize global variables.
blnTXSuccess = False
gnFrameCount = 0                        # Counter to keep track of the number of frames processed.
pwmLED1 = PWMLED(24)                    # Initialize GPIO24 to control LED.
pwmLED1.value = _LED_NORMAL_BRIGHTNESS  # Turn on LED at normal brigthness.
# Load database
face_cascade = cv2.CascadeClassifier('/home/pi/opencv-4.1.0/data/haarcascades/haarcascade_frontalface_default.xml')

# --- Image Processing Algorithm 1 - Face Recognition and Tracking ---
def IPA1_FaceRecognitionTracking():
# In this infinite loop, after each image frame is obtained from the camera buffer, Haar cascade inference
# is performed to see if any region-of-interest (ROI) containing human face. If face is present, it's (x,y)
# coordinate for the ROI will be transmitted via serial port for the 1st face in the frame. If no face is
# detected, an (x,y) coordinate of (255,255) will be send out. Thus, whether the current image frame contains
# human face or not, the serial port will always transmit a packet of bytes out for each frame. This feature
# can be used to check the actual frame rate of the system.

    # Declare global variables.
    global gnFrameCount
    
    img = video.read()
    nFaceCount = 0                                          # Reset face counter, counter to keep track of number
                                                            # faces detected per frame.
    imgGray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)          # Convert original RGB color image to grayscale.
    faces = face_cascade.detectMultiScale(imgGray,1.1,5)    # Here a scale of 1.1 is used. The image is reduced
                                                            # in size by 10% for every iteration.
    for (x,y,w,h) in faces:
        gnFrameCount = 0                                    # Reset the frame counter.
        nFaceCount = nFaceCount + 1                         # Increment face counter.                                                     
        if nFaceCount == 1:                                 # Only transmitfor the 1st face detected.
            pwmLED1.value = _LED_MAX_BRIGHTNESS             # Set LED to full brightness.
            img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2) # Draw a blue rectangle across 1st
                                                            # face region detected.
            xerror = _FRAME_WIDTH_DIV2 - (x+(w>>1))         # Calculate deviation from center of frame.
                                                            # Bitwise operation is used to speed things up.
            # Send instructions to drive external motors to point the camera towards a face.
            
            xerror = xerror >> 4                            # Scale the error and convert to integer, else
                                                            # cannot send via serial port.
            #print(xerror)
            if xerror > 0:
                if xerror > 9:                              # Limit the magnitude to 9.
                    xerror = 9
                blnTXSuccess = HeadRC_Interface.sendtoRC(bytes([100,48,45,48+xerror,48,10]))
                                                            #"d0-exrror0\n" Turn smart servo.
                #print(xerror)
            elif xerror < 0:
                if xerror < -9:                             # Limit the magnitude to 9.
                    xerror = -9
                blnTXSuccess = HeadRC_Interface.sendtoRC(bytes([100,48,43,48-xerror,48,10]))
                                                            #"d0+xerror0\n" Turn smart servo.
                #print(-xerror)
                            
            yerror = _FRAME_HEIGHT_DIV2 - (y+(h>>1))        # Calculate deviation from center of frame.
                                                            # Bitwise operation is used to speed things up.
            yerror = yerror >> 4                            # Scale the error and convert to integer.
            #print(yerror)

            nTXErrorCount = 0                               # Reset error counter. Not used for now.
            if yerror > 0:
                if yerror > 9:                              # Limit the magnitude to 9.
                    yerror = 9
                while HeadRC_Interface.sendtoRC(bytes([68,48,45,48+yerror,48,10])) == False:
                    nTXErrorCount = nTXErrorCount + 1       # Update error counter, not used for now.
                                                            #"D0-yerror0\n" Turn RC servo. Differential.
                #print(yerror)
            elif yerror < 0:
                if yerror < -9:                             # Limit the magnitude to 9.
                    yerror = -9
                while HeadRC_Interface.sendtoRC(bytes([68,48,43,48-yerror,48,10])) == False:
                    nTXErrorCount = nTXErrorCount + 1       # Update error counter, not used for now.
                                                            #"D0+yerror0\n" Turn RC servo. Differential.
                #print(-yerror)
            
    if  nFaceCount == 0:                                    # The face counter will be 0 if no face
        pwmLED1.value = _LED_NORMAL_BRIGHTNESS              # Set LED to normal brightness.
        gnFrameCount = gnFrameCount + 1                     # Increment frame counter.
        if gnFrameCount > 32:                               # If more than 32 frames without detecting faces.
            nTXErrorCount = 0                               # Reset error counter. Not used for now.
            #blnTXSuccess = HeadRC_Interface.sendtoRC(bytes([109,48,43,48,48,10])) 
            blnTXSuccess = HeadRC_Interface.sendtoRC(bytes([77,48,43,48,48,10])) #"M0+00" Turn RC servo back to default angle.
            gnFrameCount = 0
            # 1 May 2021: Because the HeadRC_Interface thread only accept 1 instruction string to the RC at
            # each invocation and an interval of 20-40msec (see source code for HeadRC_Interface thread)
            # between each invocation, the second invocation needs
            # to poll the return to ensure the request to send command string is accepted, i.e. True is
            # return.
            while HeadRC_Interface.sendtoRC(bytes([109,48,43,48,48,10])) == False: #"m0+00" Turn smart servo back to default angle.
                nTXErrorCount = nTXErrorCount + 1           # Update error counter, not used for now.
            #while HeadRC_Interface.sendtoRC(bytes([77,48,43,48,48,10])) == False:
            #    nTXErrorCount = nTXErrorCount + 1
            
    cv2.imshow("Video",img)                                 # Show camera perspective.

# --- Image Processing Algorithm 2 - Shi-Tomasi Feature Detection ---
def IPA2_FeatureDetect_ShiTomasi():
    # Declare global variables.
    global gnFrameCount

    numberCorner = 12
    
    img = video.read()
    imgGray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)          # Convert RGB color image to grayscale.
    # Return an array of floating point values that contains the 
    # detected corner coordinates.
    # The first index points to each corner, 2nd index = 0.
    # 3rd index is 0 (x coordinate) or 1 (y coordinate)
    corners = cv2.goodFeaturesToTrack(imgGray,numberCorner,0.20,10)  # Quality level = 0.20
                                                                    # A smaller quality level makes the
                                                                    # method most sensitive, hence finding
                                                                    # larger number of corners.
                                                                    # Min corner separation = 10 pixels.

    corners1 = np.int0(corners) # Convert an array of floating point values into 
                                # integers. Note that int0() is an alias of intp()

    for i in corners1:
        x,y = i.ravel()
        cv2.circle(img,(x,y),5,[0,255,0],-1) 
        
    cv2.imshow("Video",img)                                         # Show camera perspective.

# --- Image Processing Algorithm 3 - Sparse Optical Flow with Shi-Tomasi Feature Detection ---

# params for ShiTomasi corner detection
feature_params = dict( maxCorners = 15,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )

# Parameters for lucas kanade optical flow
lk_params = dict( winSize  = (11,11),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

class IPA_OpticalFlowLK():

    imgGray_old = []        # Class wide static variables or objects.
    featurePoint0 = []  
    errorflag = False                    

    
    def __init__(self):
        img_old = video.read()
        IPA_OpticalFlowLK.imgGray_old = cv2.cvtColor(img_old,cv2.COLOR_BGR2GRAY)  # Convert RGB color image to grayscale.
        IPA_OpticalFlowLK.featurePoint0 = cv2.goodFeaturesToTrack(IPA_OpticalFlowLK.imgGray_old, mask = None, **feature_params)
    '''
    def start(self):
        img_old = video.read()
        IPA_OpticalFlowLK.imgGray_old = cv2.cvtColor(img_old,cv2.COLOR_BGR2GRAY)  # Convert RGB color image to grayscale.
        IPA_OpticalFlowLK.featurePoint0 = cv2.goodFeaturesToTrack(IPA_OpticalFlowLK.imgGray_old, mask = None, **feature_params)           
    '''      
    def run(self):
        img = video.read() # Read 1 image frame from video.
        imgGray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)  # Convert RGB color image to grayscale.
   
        if IPA_OpticalFlowLK.featurePoint0 is not None:     # Make sure the old feature points (featurePoint0) are valid before
                                            # calculating the optical flow using Lukas-Kanade method, else an exception
                                            # will be triggered by the routines.   
            # Calculate optical flow using Lukas-Kanade method.
            featurePoint1, st, err = cv2.calcOpticalFlowPyrLK(IPA_OpticalFlowLK.imgGray_old, imgGray, IPA_OpticalFlowLK.featurePoint0, None, **lk_params)
    
            if st is not None:              # Only extract new feature points, plot circles, backup
                                            # current grayscale image frame etc if featurePoint1 and
                                            # st are not empty, i.e. None. Here we use the operator
                                            # 'is not' instead of the comparison operator '!=' as
                                            # None is an object that is empty. '!=' operator compare
                                            # the contents of two objects where 'is' and 'is not' 
                                            # compare the address of the object. Somehow using '!='
                                            # will leads to an exception when Python executes.
                                            # indicating the position of feature points when st 
                                            # is not empty.
                # Select good feature points.
                # st indicates the status of the feature points returned, it is a numpy array with entry of 0 or 1.
                # Entry of 1 means the corresponding featurePoint1 at the same index is good. The statement below
                # uses the masking function of numpy array, extract out all feature points corresponds to index with
                # value of 1 in the st array.
                goodfp_new = featurePoint1[st==1]    
                corners1 = np.int0(featurePoint1)   # Convert an array of floating point values into 
                                                # integers. Note that int0() is an alias of intp()        
                corners2 = np.int0(IPA_OpticalFlowLK.featurePoint0)   # Convert an array of floating point values into 
                                                # integers. Note that int0() is an alias of intp()      
                deltax = 0
                deltay = 0      
                # Get the (x,y) coordinates of old and new feature points.
                # Calculate the cumulative average difference (in pixels)
                # along x and y axes between the new and old feature points.
                for i in range(corners1.shape[0]):
                    x1,y1 = corners1[i][0]          # (x,y) coordinates of new feature point.
                    x2,y2 = corners2[i][0]          # (x,y) coordinates of old feature point.
                    #cv2.circle(img,(x1,y1),4,[0,255,0],-1) # Draw green circle for new feature points.
                    #cv2.circle(img,(x2,y2),4,[0,0,255],-1) # Draw blue circle for old feature points.
                    deltax = deltax + (x2-x1)       # Update the cumulative difference along x axis.
                    deltay = deltay + (y2-y1)       # Update the cumulative difference along y axis.
                ave_deltax = (2*deltax)/(i+1)       # Calculate average x difference between all feature points.
                ave_deltay = (2*deltay)/(i+1)   # Calculate average y difference between all feature points.
                cv2.line(img,(160,100),(160,100+np.int0(ave_deltay)),(0,255,0),8)  # Green line for y change.
                cv2.line(img,(160,100),(160+np.int0(ave_deltax),100),(0,255,255),8) # Yellow line for x change.
                # direction arrow.
                cv2.circle(img,(160,100),4,[255,0,0],-1) # Draw red circle to indicate the center of the arrow..    
                # Carry over new feature points.
                # featurePoint0 = goodfp_new.reshape(-1,1,2)
                # print("No. of feature points = %d" % featurePoint1.shape[0]) 
                # Check if we still have sufficient feature points to compute the optical flow in the 
                # next sequence, else rerun the feature extraction algorithm to find a fresh set of
                # feature points from the old frame.
                if featurePoint1.shape[0] < 6:      # if less than 6 feature points.
                    IPA_OpticalFlowLK.featurePoint0 = cv2.goodFeaturesToTrack(IPA_OpticalFlowLK.imgGray_old, mask = None, **feature_params)
                else:
                    IPA_OpticalFlowLK.featurePoint0 = goodfp_new.reshape(-1,1,2)   

            else: #if st is not None
             # Get new feature points from current frame.
                 IPA_OpticalFlowLK.featurePoint0 = cv2.goodFeaturesToTrack(imgGray, mask = None, **feature_params)
             # Update the old frame.
            IPA_OpticalFlowLK.imgGray_old = imgGray    
        
        else: #if featurePoint0 is not None:
            # The previous attempt did not generate any good feature points. Try to get features again using
            # current frame. But first backup the old frame.
            IPA_OpticalFlowLK.imgGray_old = imgGray
            IPA_OpticalFlowLK.featurePoint0 = cv2.goodFeaturesToTrack(imgGray, mask = None, **feature_params)
 
        cv2.imshow("Video",img)           # Display the image frame.  

    
# Setup the parameters of serial port
sport = serial.Serial()
sport.baudrate = 115200
sport.port = '/dev/ttyS0' # Tested ok for raspberry pi 3B and 3A+
sport.stopbits = serial.STOPBITS_ONE
sport.bytesize = serial.EIGHTBITS
sport.timeout = 0.02  # Read timeout value, 0.02 seconds or 20 msec.
                     # The external controller typically responds within 4 msec
sport.write_timeout = 0.05    # write timeout value, 0.05 second.
sport.xonxoff = False  # No hardware and software hand-shaking.
sport.dsrdtr = False
sport.rtscts = False

# Open the serial port.
sport.open()
if sport.is_open == True:
    print("Port %s is opened" % sport.port)
else:
    print("Port %s not opened" % sport.port)
    
# Create and start new thread
HeadRC_Interface = SerialCommThread("RC Comm Handler")      # Robot Controller interface thread.
HeadRC_Interface.start()
video = CamThread((_FRAME_WIDTH,_FRAME_HEIGHT),_FRAME_RATE).start() # Pi Camera interface thread.

time.sleep(2.0)                 # Allow the camera to warmup and start.

gnFrameCount = 0

ipa3 = IPA_OpticalFlowLK()

while True:
    #IPA1_FaceRecognitionTracking()
    #IPA2_FeatureDetect_ShiTomasi()
    ipa3.run()                               # Run IPA routines
    if cv2.waitKey(1) & 0xFF == ord('q'):
          
        break

video.stop() 
HeadRC_Interface.stop()     # Stop all thread
cv2.destroyAllWindows()     # Close active window associated with main thread.
#sport.close()               # Release resources associated with serial port.   
pwmLED1.off()               # Turn off LED to indicate that program is no longer executing.
