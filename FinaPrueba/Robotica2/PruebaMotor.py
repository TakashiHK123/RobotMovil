import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BCM)

MotorIN1 = 15
MotorIN2 = 14
MotorE1 = 18

GPIO.setup(MotorIN1,GPIO.OUT)
GPIO.setup(MotorIN2,GPIO.OUT)
GPIO.setup(MotorE1,GPIO.OUT)

print("Hacemos girar el motor en un sentido por 5 segundos")
GPIO.output(MotorIN1,GPIO.HIGH) # Establecemos el sentido de giro con los pines IN1 e IN2  
GPIO.output(MotorIN2,GPIO.LOW)  # Establecemos el sentido de giro con los pines IN1 e IN2
GPIO.output(MotorE1,GPIO.HIGH)  # Habilitamos las salidas OUT1 y OUT2 del puente H

sleep(1)

print ("Detenemos el motor")
GPIO.output(MotorE1,GPIO.LOW)

sleep(1)

print("Hacemos girar el motor en el sentido contrario por 5 segundos")
GPIO.output(MotorIN1,GPIO.LOW)   # Establecemos el sentido de giro con los pines IN1 e IN2   
GPIO.output(MotorIN2,GPIO.HIGH)  # Establecemos el sentido de giro con los pines IN1 e IN2 
GPIO.output(MotorE1,GPIO.HIGH)   # Habilitamos las salidas OUT1 y OUT2 del puente H

sleep(1)

print ("Detenemos el motor")
GPIO.output(MotorE1,GPIO.LOW)

sleep(1)

GPIO.cleanup()
