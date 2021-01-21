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

"""

if not video.isOpened():        # Check if video channel is opened.
    print("Cannot open camera")
    exit()

while True:                     # Continuously display video image on window until 'q' key is pressed.
    flag, img = video.read()    # Get 1 frame from video buffer.
    if not flag:
        print("Can't receive frame, exiting...")
        break

    cv2.imshow("Video",img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
"""
for (i,f) in enumerate(stream):
    frame = f.array
    cv2.imshow("Video", frame)
    rawCapture.truncate(0) # Clear the stream in preparation for next frame.
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break    

cv2.destroyAllWindows()
stream.close()
rawCapture.close()
camera.close()

