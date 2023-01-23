import threading
from gpiozero import DigitalInputDevice
from time import sleep
import time
import asyncio

import RPi.GPIO as GPIO

class Encoder(object):
    def __init__(self, pin):
        self._value = 0
        encoder = DigitalInputDevice(pin, pull_up=True)
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

v_sonido = 34300 #cm/s velocidad del sonido

TRIGM = 23 #Variable que contiene el GPIO al cual conectamos la señal TRIG del sensor
ECHOM = 24 #Variable que contiene el GPIO al cual conectamos la señal ECHO del sensor

TRIGR = 25
ECHOR = 5

TRIGL = 22
ECHOL = 27

MotorIN1 = 15
MotorIN2 = 14
MotorE1 = 13
MotorDIN1 = 3
MotorDIN2 = 4
MotorE2 = 12

SAMPLETIME = 0.5
TARGET = 33
KP = 0.02
KD = 0.01
KI = 0.005





GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIGM, GPIO.OUT) #Configuramos el pin TRIG como una salida 
GPIO.setup(ECHOM, GPIO.IN)  #Configuramos el pin ECHO como una salida 
GPIO.setup(TRIGR, GPIO.OUT)
GPIO.setup(ECHOR, GPIO.IN)
GPIO.setup(TRIGL, GPIO.OUT)
GPIO.setup(ECHOL, GPIO.IN)
GPIO.setup(MotorIN1,GPIO.OUT)
GPIO.setup(MotorIN2,GPIO.OUT)
GPIO.setup(MotorE1,GPIO.OUT)
GPIO.setup(MotorDIN1,GPIO.OUT)
GPIO.setup(MotorDIN2,GPIO.OUT)
GPIO.setup(MotorE2,GPIO.OUT)
GPIO.output(TRIGM, False)
GPIO.output(TRIGR, False)
GPIO.output(TRIGL, False)

forwardL=GPIO.PWM(MotorIN2,100) # configuring Enable pin for PWM
reverseL=GPIO.PWM(MotorIN1,100) # configuring Enable pin for PWM
forwardR=GPIO.PWM(MotorDIN2,100) # configuring Enable pin for PWM
reverseR=GPIO.PWM(MotorDIN1,100)

def motorPwm(left,right):
    if left==0 and right==0:
        forwardL.stop() # stop PWM from GPIO output it is necessary
        reverseL.stop()
        forwardR.stop() # stop PWM from GPIO output it is necessary
        reverseR.stop()
        
    forwardL.ChangeDutyCycle(left)
    reverseL.ChangeDutyCycle(0)
    forwardR.ChangeDutyCycle(right)
    reverseR.ChangeDutyCycle(0)
    
def initMotor():
    forwardL.start(0) 
    reverseL.start(0)
    forwardR.start(0) 
    reverseR.start(0)
    GPIO.output(MotorE1,GPIO.HIGH)
    GPIO.output(MotorE2,GPIO.HIGH)  
    
def convertidor(dato):
    dato=dato*100
    return dato
    
def condicionParada(num):
    if 10 > num:
        print("Now stop")
        motorPwm(0,0)
        time.sleep(2)
            
            
def medirDistancia(ECHO, TRIG, distanciaAnterior): 
        GPIO.output(TRIG, True) #Trig sensor medio alto
        time.sleep(0.00001) #Delay de 100 microsegundos
        GPIO.output(TRIG, False) #Trig sensor medio bajo
        
        while GPIO.input(ECHO)==0:
            inicio_pulso = time.time()
            
        while GPIO.input(ECHO)==1:
            fin_pulso = time.time()
        
        tiempo = fin_pulso - inicio_pulso
        distancia = v_sonido * (tiempo/2)
        distancia = round(distancia,2)
        
        if distancia > 2 and distancia < 400:
            return distancia
        else:
            return distanciaAnterior
        #await uasyncio.sleep_ms(100)
        

def imprimirDistancia(result, sensor):
    print("Distancia ",sensor,": ",result, " cm")

e1 = Encoder(17)
e2 = Encoder(26)


m1_speed = 0.5
m2_speed = 0.5

e1_prev_error = 0
e2_prev_error = 0

e1_sum_error = 0
e2_sum_error = 0
distanciaM=100
distanciaR=100
distanciaL=100
initMotor()
motorPwm(50,50)
while True:
   
    e1_error = TARGET - e1.value
    e2_error = TARGET - e2.value
    print("Encoder I:",e1_error)
    print("Encoder D:", e2_error)
    m1_speed += (e1_error * KP) + (e1_prev_error * KD) + (e1_sum_error * KI)
    m2_speed += (e2_error * KP)  + (e1_prev_error * KD) + (e2_sum_error * KI)

    m1_speed = max(min(1, m1_speed), 0)
    m2_speed = max(min(1, m2_speed), 0)
    m1_speed=convertidor(m1_speed)
    m2_speed=convertidor(m2_speed)
    
    motorPwm(m1_speed,m2_speed)

    print("e1 {} e2 {}".format(e1.value, e2.value))
    print("m1 {} m2 {}".format(m1_speed, m2_speed))

    e1.reset()
    e2.reset()

    sleep(SAMPLETIME)

    e1_prev_error = e1_error
    e2_prev_error = e2_error

    e1_sum_error += e1_error
    e2_sum_error += e2_error
    
    distanciaM = medirDistancia(ECHOM, TRIGM,distanciaM)
    #imprimirDistancia(distanciaM,"central")
    condicionParada(distanciaM)
    
    distanciaR = medirDistancia(ECHOR, TRIGR,distanciaR)
    #imprimirDistancia(distanciaR,"derecha")
    condicionParada(distanciaR)
    
    distanciaL = medirDistancia(ECHOL, TRIGL,distanciaL)
    #imprimirDistancia(distanciaL,"izquierda")
    condicionParada(distanciaL)
#asyncio.run(main())
