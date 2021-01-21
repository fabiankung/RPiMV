import cv2

face_cascade = cv2.CascadeClassifier('/home/pi/opencv-4.1.0/data/haarcascades/haarcascade_frontalface_default.xml')

video = cv2.VideoCapture(0)     # Open a default camera connected to computer.
video.set(3,640)                # Set video resolution to VGA, i.e. 640x480 pixels.
video.set(4,480)

if not video.isOpened():        # Check if video channel is opened.
    print("Cannot open camera")
    exit()

while True:                     # Continuously display video image on window until 'q' key is pressed.
    flag, img = video.read()    # Get 1 frame from video buffer.
    if not flag:
        print("Can't receive frame, exiting...")
        break

    imgGray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)  # Convert original RGB color image to grayscale.
    faces = face_cascade.detectMultiScale(imgGray,1.1,3)

    for (x,y,w,h) in faces:
        img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),3) # Draw a blue rectangle across each
                                                             # face region detected.
    cv2.imshow("Video",img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

video.release()
cv2.destroyAllWindows()
