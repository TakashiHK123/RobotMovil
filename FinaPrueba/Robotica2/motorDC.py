import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BCM)

MotorIN1 = 15
MotorIN2 = 14
MotorE1 = 18
MotorDIN1 = 3
MotorDIN2 = 4
MotorE2 = 21
ENCODERL = 17
ENCODERR = 26


GPIO.setup(MotorIN1,GPIO.OUT)
GPIO.setup(MotorIN2,GPIO.OUT)
GPIO.setup(MotorDIN1,GPIO.OUT)
GPIO.setup(MotorDIN2,GPIO.OUT)
GPIO.setup(MotorE1,GPIO.OUT)
GPIO.setup(MotorE2,GPIO.OUT)
#GPIO.setup(ENCODERL, GPIO.IN)
#GPIO.setup(ENCODERR, GPIO.IN)
print("Hacemos girar el motor en un sentido por 5 segundos")
GPIO.output(MotorIN2,GPIO.HIGH) # Establecemos el sentido de giro con los pines IN1 e IN2
GPIO.output(MotorIN1,GPIO.LOW)  # Establecemos el sentido de giro con los pines IN1 e IN2
GPIO.output(MotorDIN2,GPIO.HIGH)
GPIO.output(MotorDIN1,GPIO.LOW)

GPIO.output(MotorE1,GPIO.HIGH)  # Habilitamos las salidas OUT1 y OUT2 del puente H para los dos motores
GPIO.output(MotorE2,GPIO.HIGH)
sleep(5)

print ("Detenemos el motor")
GPIO.output(MotorE1,GPIO.LOW)
GPIO.output(MotorE2,GPIO.LOW)
sleep(1)
while True:
    GPIO.output(MotorE2,GPIO.LOW)

GPIO.cleanup()