import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray

#import imutils
import time


camera = PiCamera()
camera.resolution = (320,240)
camera.framerate = 24
rawCapture = PiRGBArray(camera, size=(320,240))
stream = camera.capture_continuous(rawCapture, format="bgr", use_video_port=True)

time.sleep(2.0)                 # Allow the camera to warmup and start.
face_cascade = cv2.CascadeClassifier('/home/pi/opencv-4.1.0/data/haarcascades/haarcascade_frontalface_default.xml')

"""
    imgGray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)  # Convert original RGB color image to grayscale.
    faces = face_cascade.detectMultiScale(imgGray,1.1,3)

    for (x,y,w,h) in faces:
        img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),3) # Draw a blue rectangle across each
                                                             # face region detected.
    cv2.imshow("Video",img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
"""

for (i,f) in enumerate(stream):
    img = f.array     # Get RGB array.
    imgGray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)  # Convert original RGB color image to grayscale.
    
    faces = face_cascade.detectMultiScale(imgGray,1.1,3)

    for (x,y,w,h) in faces:
        img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),3) # Draw a blue rectangle across each
                                                             # face region detected.
    cv2.imshow("Video", img)
    
    rawCapture.truncate(0) # Clear the stream in preparation for next frame.
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
cv2.destroyAllWindows()
stream.close()
rawCapture.close()
camera.close()
