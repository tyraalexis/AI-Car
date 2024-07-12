import time
from Motor import *
import RPi.GPIO as GPIO

class Line_Tracking:
    def __init__(self):
        self.IR01 = 14
        self.IR02 = 15
        self.IR03 = 23
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.IR01, GPIO.IN)
        GPIO.setup(self.IR02, GPIO.IN)
        GPIO.setup(self.IR03, GPIO.IN)

    def run(self):
        while True:
            self.LMR = 0x00
            if not GPIO.input(self.IR01):  # Detecting white surface
                self.LMR |= 4
            if not GPIO.input(self.IR02):  # Detecting white surface
                self.LMR |= 2
            if not GPIO.input(self.IR03):  # Detecting white surface
                self.LMR |= 1

            if self.LMR == 7:  # Straight
                PWM.setMotorModel(800, 800, 800, 800)
            elif self.LMR == 3:  # Turn slightly right
                PWM.setMotorModel(1500, 1500, -1500, -1500)
            elif self.LMR == 1:  # Turn right
                PWM.setMotorModel(2000, 2000, -2000, -2000)
            elif self.LMR == 6:  # Turn slight left
                PWM.setMotorModel(-1500, -1500, 1500, 1500)
            elif self.LMR == 4:  # Turn left
                PWM.setMotorModel(-2000, -2000, 2000, 2000)
            else:  # LMR = 0, All sensors on black (boundary detected)
                PWM.setMotorModel(-800, -800, -800, -800)  # Back up
                time.sleep(0.5)  # Back up for 0.5 seconds
                PWM.setMotorModel(-1500, -1500, 1500, 1500)
                time.sleep(.3)
                PWM.setMotorModel(800, 800, 800, 800)
                if self.LMR == 0:
                    PWM.setMotorModel(1500, 1500, -1500, -1500)
                    time.sleep(.3)
                    PWM.setMotorModel(800, 800, 800, 800)
            time.sleep(0.01)  # Add a small delay to prevent high CPU usage
            continue
infrared = Line_Tracking()

if __name__ == '__main__':
    print('Program is starting ... ')
    try:
        infrared.run()
    except KeyboardInterrupt:
        PWM.setMotorModel(0, 0, 0, 0)
        GPIO.cleanup()  # Cleanup GPIO settings
