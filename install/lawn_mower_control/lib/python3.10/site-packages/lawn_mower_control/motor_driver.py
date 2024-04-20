import RPi.GPIO as GPIO
import numpy as np

r_in1 = 24
r_in2 = 25
r_en = 12

l_in1 = 5
l_in2 = 6
l_en = 13

GPIO.setmode(GPIO.BCM)
GPIO.setup(r_in1, GPIO.OUT)
GPIO.setup(r_in2, GPIO.OUT)
GPIO.setup(r_en, GPIO.OUT)
GPIO.setup(l_in1, GPIO.OUT)
GPIO.setup(l_in2, GPIO.OUT)
GPIO.setup(l_en, GPIO.OUT)

GPIO.output(r_in1, GPIO.LOW)
GPIO.output(r_in2, GPIO.LOW)
GPIO.output(l_in1, GPIO.LOW)
GPIO.output(l_in2, GPIO.LOW)
class Motor():
    def __init__(self, in1, in2, en):
        self.in1 = in1
        self.in2 = in2
        self.en = en
        GPIO.setup(in1, GPIO.OUT)
        GPIO.setup(in2, GPIO.OUT)
        GPIO.setup(en, GPIO.OUT)

        self.pwm = GPIO.PWM(en, 1000)
        self.pwm.start(0)
        self.stop()

    def stop(self):
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)
    
    def forward(self):
        GPIO.output(self.in1, GPIO.HIGH)
        GPIO.output(self.in2, GPIO.LOW)

    def backward(self):
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.HIGH)

    def set_speed(self, speed):
        """
        speed is in (0, 1] where 0 is off and 1 is full speed
        """
        duty_cycle = int(speed * 50 + 50)
        if duty_cycle < 0 or duty_cycle > 100:
            print("duty cycle out of range: {}".format(duty_cycle))
            duty_cycle = max(0, min(100, duty_cycle))
        self.pwm.ChangeDutyCycle(duty_cycle)

class MotorDriver:
    def __init__(self):
        self.left_wheel = Motor(l_in1, l_in2, l_en)
        self.right_wheel = Motor(r_in1, r_in2, r_en)

    def update(self, left_speed, right_speed):
        log_str = ""
        log_str += self.update_wheel(left_speed, True) + '\n'
        log_str += self.update_wheel(right_speed, False)
        return log_str

    def update_wheel(self, speed, left_wheel=False):
        wheel = self.left_wheel if left_wheel else self.right_wheel
        if speed == 0:
            wheel.stop()
            return "Stopping {} wheel".format('left' if left_wheel else 'right')
        elif speed > 0:
            wheel.forward()
        else:
            wheel.backward()
        speed = abs(np.round(speed))
        wheel.set_speed(speed)
        return "Setting {} wheel {} at speed {}".format(
                    'left' if left_wheel else 'right'
                    , 'forward' if speed > 0 else 'backward'
                    , speed)