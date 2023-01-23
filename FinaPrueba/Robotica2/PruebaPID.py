import threading
from gpiozero import DigitalInputDevice
from time import sleep
import RPi.GPIO as GPIO

class Encoder(object):
    def __init__(self, pin):
        self._value = 0

        encoder = DigitalInputDevice(pin)
        encoder.when_activated = self._increment
        encoder.when_deactivated = self._increment
        
    def reset(self):
        self._value = 0

    def _increment(self):
        self._value += 1

    @property
    def value(self):
        return self._value

def clamp(value):
    return max(min(1, value), 0)

SAMPLETIME = 0.5
TARGET = 15
KP = 0.02
KD = 0.01
KI = 0.005
 
MotorIN1 = 15
MotorIN2 = 14
MotorE1 = 18
MotorDIN1 = 3
MotorDIN2 = 4
MotorE2 = 21

GPIO.setmode(GPIO.BCM)
GPIO.setup(MotorIN1,GPIO.OUT)
GPIO.setup(MotorIN2,GPIO.OUT)
GPIO.setup(MotorE1,GPIO.OUT)
GPIO.setup(MotorDIN1,GPIO.OUT)
GPIO.setup(MotorDIN2,GPIO.OUT)
GPIO.setup(MotorE2,GPIO.OUT)


m1_speed = 1
m2_speed = 1

e1_prev_error = 0
e2_prev_error = 0

e1_sum_error = 0
e2_sum_error = 0

reverse=GPIO.PWM(MotorIN1,100) # configuring Enable pin for PWM
forward=GPIO.PWM(MotorIN2,100) # configuring Enable pin for PWM
reverse2=GPIO.PWM(MotorDIN1,100) # configuring Enable pin for PWM
forward2=GPIO.PWM(MotorDIN2,100)

while True:

    forward.start(0) 
    reverse.start(0)
    forward2.start(0) 
    reverse2.start(0)
    
    e1_error = TARGET - e1.value
    e2_error = TARGET - e2.value

    m1_speed += (e1_error * KP) + (e1_prev_error * KD) + (e1_sum_error * KI)
    m2_speed += (e2_error * KP)  + (e1_prev_error * KD) + (e2_sum_error * KI)

    m1_speed = max(min(1, m1_speed), 0)
    m2_speed = max(min(1, m2_speed), 0)
    
    GPIO.output(MotorE1,GPIO.HIGH)
    GPIO.output(MotorE2,GPIO.HIGH)
    forward.ChangeDutyCycle(100)
    reverse.ChangeDutyCycle(0)
    forward2.ChangeDutyCycle(100)
    reverse2.ChangeDutyCycle(0)


    print("e1 {} e2 {}".format(e1.value, e2.value))
    print("m1 {} m2 {}".format(m1_speed, m2_speed))

    e1.reset()
    e2.reset()

    sleep(SAMPLETIME)

    e1_prev_error = e1_error
    e2_prev_error = e2_error

    e1_sum_error += e1_error
    e2_sum_error += e2_error