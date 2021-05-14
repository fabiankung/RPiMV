
import cv2
import time
from gpiozero import PWMLED

# NOTE: 12 APril 2021
# the prf (pulse repetitive frequency) for software PWM is around 100 Hz
# from oscilloscope measurement.
pwmLED2 = PWMLED(24)  # Initialize GPIO24 to control LED via software PWM.
pwmLED = PWMLED(18)  # Initialize GPIO18 to control LED via software PWM.

while True:
    pwmLED.value = 0.1
    pwmLED2.value = 1.0
    time.sleep(0.5)
    pwmLED.value = 0.3
    pwmLED2.value = 0.9
    time.sleep(0.5)
    pwmLED.value = 0.5
    pwmLED2.value = 0.7
    time.sleep(0.5)
    pwmLED.value = 0.7
    pwmLED2.value = 0.5
    time.sleep(0.5)
    pwmLED.value = 0.9
    pwmLED2.value = 0.3
    time.sleep(0.5)
    pwmLED.value = 1.0
    pwmLED2.value = 0.1
    time.sleep(0.5)
 
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        video.stop()
        break

pwmLED.off()
