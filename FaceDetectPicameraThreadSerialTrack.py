"""
Codes to:
1. Enable multi-threading.
2. Open raspiberry pi camera - picamera using native interface in a thread.
   Default camera resolution is 320x240 pixels. User can change to 640x480
   or 160x120 (Note: we can also rescale the image and print it
   out at larger dimension).
3. Run face detection algorithim using Haar cascades method in OpenCV2.
4. Enable pyserial for communication with external Robot Controller (RC).
5. Turn on GPIO12 pin to drive external LED.
6. Send instruction to turn robot platform elevation and azimuth motor so
   that the pi-camera can track the face.
7. To quit, press the key 'q' on the keyboard. 

11/4/2021
Fabian Kung
"""

import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray
from threading import Thread
import serial
import time
from gpiozero import PWMLED

_FRAME_WIDTH = 320  # In pixels.
_FRAME_HEIGHT = 240 # In pixels.
_FRAME_WIDTH_DIV2 = 160 # In pixels.
_FRAME_HEIGHT_DIV2 = 120 # In pixels.
_FRAME_RATE = 24    # In frames-per-second.

_LED_NORMAL_BRIGHTNESS = 0.1  # Normal brightness of LED, 10%.
_LED_MAX_BRIGHTNESS = 0.9

# Define camera class
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
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        # Keep looping indefinitely until the thread is stopped.
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

# Setup the parameters of serial port.
sport = serial.Serial()
sport.baudrate = 115200
sport.stopbits = serial.STOPBITS_ONE
sport.bytesize = serial.EIGHTBITS
sport.timeout = 0.02 # Read timeout value, 0.02 seconds or 20 msec.
                     # The external controller typically responds within 4 msec.
sport.write_timeout = 0.1 # write timeout value, 0.1 second.
sport.xonxoff = False
sport.dsrdtr = False
sport.rtscts = False
sport.port = '/dev/ttyS0' # Tested ok for raspberry pi 3B and 3A+

# Open the serial port.
sport.open()
if sport.is_open:
    print("Port %s is opened" % sport.port)
else:
    print("Port is not opened!")
        
#video = CamThread((160,120),10)
#video.start()
video = CamThread((_FRAME_WIDTH,_FRAME_HEIGHT),_FRAME_RATE).start()
time.sleep(2.0)                 # Allow the camera to warmup and start.
face_cascade = cv2.CascadeClassifier('/home/pi/opencv-4.1.0/data/haarcascades/haarcascade_frontalface_default.xml')

gnFrameCount = 0  # Counter to keep track of the number of frames processed.
gnFaceCount = 0   # Counter to keep track of the number of faces detected per frame.

pwmLED1 = PWMLED(24) # Initialize GPIO124 to control LED.
pwmLED1.value = _LED_NORMAL_BRIGHTNESS # Turn on LED at normal brigthness.

# In this infinite loop, after each image frame is obtained from the camera buffer, Haar cascade inference
# is performed to see if any region-of-interest (ROI) containing human face. If face is present, it's (x,y)
# coordinate for the ROI will be transmitted via serial port for the 1st face in the frame. If no face is
# detected, an (x,y) coordinate of (255,255) will be send out. Thus, whether the current image frame contains
# human face or not, the serial port will always transmit a packet of bytes out for each frame. This feature
# can be used to check the actual frame rate of the system.
while True:
    img = video.read()
    gnFaceCount = 0  # Reset face counter
    imgGray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)           # Convert original RGB color image to grayscale.
    faces = face_cascade.detectMultiScale(imgGray,1.1,5)     # Here a scale of 1.1 is used. The image is reduced
                                                             # in size by 10% for every iteration.
    for (x,y,w,h) in faces:
        gnFrameCount = 0                                     # Reset the frame counter.
        gnFaceCount = gnFaceCount + 1                        # Increment face counter.                                                     
        if gnFaceCount == 1:                                 # Only transmitfor the 1st face detected.
            pwmLED1.value = _LED_MAX_BRIGHTNESS             # Set LED to full brightness.
            img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2) # Draw a blue rectangle across 1st
                                                             # face region detected.
            xerror = _FRAME_WIDTH_DIV2 - (x+(w>>1))          # Calculate deviation from center of frame.
                                                             # Bitwise operation is used to speed things up.
            # Send instructions to drive external motors to point the camera towards a face.
            
            xerror = xerror >> 4                             # Scale the error and convert to integer, else
                                                             # cannot send via serial port.
            #print(xerror)
            if xerror > 0:
                if xerror > 9:                               # Limit the magnitude to 9.
                    xerror = 9
                txcount = sport.write(bytes([100,48,45,48+xerror,48,10])) #"d0-exrror0\n" Turn smart servo.
                #print(xerror)
            elif xerror < 0:
                if xerror < -9:                              # Limit the magnitude to 9.
                    xerror = -9
                txcount = sport.write(bytes([100,48,43,48-xerror,48,10])) #"d0+xerror0\n" Turn smart servo.
                #print(-xerror)
            readData = sport.read(2)                         # Read 2 bytes from RC, "OK".
                
            yerror = _FRAME_HEIGHT_DIV2 - (y+(h>>1))         # Calculate deviation from center of frame.
                                                             # Bitwise operation is used to speed things up.
            yerror = yerror >> 4                             # Scale the error and convert to integer.
            #print(yerror)

            if yerror > 0:
                if yerror > 9:                               # Limit the magnitude to 9.
                    yerror = 9
                txcount = sport.write(bytes([68,48,45,48+yerror,48,10])) #"D0-yerror0\n" Turn RC servo. Differential. 
                #print(yerror)
            elif yerror < 0:
                if yerror < -9:                              # Limit the magnitude to 9.
                    yerror = -9
                txcount = sport.write(bytes([68,48,43,48-yerror,48,10])) #"D0+yerror0\n" Turn RC servo. Differential.
                #print(-yerror)
            readData = sport.read(2)                         # Read 2 bytes from RC, "OK"
            sport.reset_input_buffer()                       # Flush serial port input buffer.
    if gnFaceCount == 0:                                     # The face counter will be 0 if no face
        #txcount = sport.write(bytes([1,255,255]))            # is detected. Send (x,y) = (255,255).
        pwmLED1.value = _LED_NORMAL_BRIGHTNESS              # Set LED to normal brightness.
        gnFrameCount = gnFrameCount + 1
        if gnFrameCount > 32:                                # If more than 32 frames without detecting faces.
            txcount = sport.write(bytes([77,48,43,48,48,10])) #"M0+00" Turn RC servo back to default angle.
            readData = sport.read(2)                         # Read 2 bytes from RC, "OK".
            gnFrameCount = 0
            txcount = sport.write(bytes([109,48,43,48,48,10])) #"m0+00" Turn smart servo back to default angle.
            readData = sport.read(2)                         # Read 2 bytes from RC, "OK".
            sport.reset_input_buffer()                       # Flush serial port           

    #img = cv2.resize(img, None, fx=2, fy=2, interpolation = cv2.INTER_LINEAR) # Increase the image size.    
    #img = cv2.resize(img, None, fx=2, fy=2, interpolation = cv2.INTER_NEAREST) # Coarser image but faster
                                                             # in displaying.
    cv2.imshow("Video", img)                                 # Show camera perspective.

    if cv2.waitKey(1) & 0xFF == ord('q'):
        video.stop()
        break

    
    #gnFrameCount = gnFrameCount + 1                        # Debug. 
    #print (gnFrameCount) 

cv2.destroyAllWindows()     # Close active window associated with main thread.
sport.close()               # Release resources associated with serial port.   
pwmLED1.off()               # Turn off LED to indicate that program is no longer executing.
