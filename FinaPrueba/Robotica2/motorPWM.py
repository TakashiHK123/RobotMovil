import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BCM)

MotorIN1 = 15
MotorIN2 = 14
MotorE1 = 13
MotorDIN1 = 3
MotorDIN2 = 4
MotorE2 = 12
ENCODERL = 13
ENCODERR = 12

GPIO.setup(MotorIN1,GPIO.OUT)
GPIO.setup(MotorIN2,GPIO.OUT)
GPIO.setup(MotorE1,GPIO.OUT)
GPIO.setup(MotorDIN1,GPIO.OUT)
GPIO.setup(MotorDIN2,GPIO.OUT)
GPIO.setup(MotorE2,GPIO.OUT)


forward=GPIO.PWM(MotorIN1,100) # configuring Enable pin for PWM
reverse=GPIO.PWM(MotorIN2,100) # configuring Enable pin for PWM
forward2=GPIO.PWM(MotorDIN1,100) # configuring Enable pin for PWM
reverse2=GPIO.PWM(MotorDIN2,100)
if __name__ == '__main__' :
    forward.start(0) 
    reverse.start(0)
    forward2.start(0) 
    reverse2.start(0)
    # this will run your motor in reverse direction for 2 seconds with 80% speed by supplying 80% of the battery voltage
    print("GO backward")
    GPIO.output(MotorE1,GPIO.HIGH)
    GPIO.output(MotorE2,GPIO.HIGH)
    
    forward.ChangeDutyCycle(0)
    reverse.ChangeDutyCycle(10)
    forward2.ChangeDutyCycle(0)
    reverse2.ChangeDutyCycle(10)
    sleep(1)
    
    #stop motor
    print("Now stop")
    #GPIO.output(MotorE1,GPIO.LOW)
    forward.stop() # stop PWM from GPIO output it is necessary
    reverse.stop()
    forward2.stop() # stop PWM from GPIO output it is necessary
    reverse2.stop()
    #while True:
        #GPIO.output(MotorE2,GPIO.LOW)
        #GPIO.output(MotorE1,GPIO.LOW)
        #GPIO.cleanup()
    