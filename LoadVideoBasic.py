import cv2

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

    cv2.imshow("Video",img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

video.release()
cv2.destroyAllWindows()
