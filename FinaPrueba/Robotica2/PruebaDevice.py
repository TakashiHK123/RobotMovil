import threading
from gpiozero import DigitalInputDevice, Robot
from time import sleep
import time

import RPi.GPIO as GPIO


def clamp(value):
    return max(min(1, value), 0)

v_sonido = 34300 #cm/s velocidad del sonido

TRIGM = 23 #Variable que contiene el GPIO al cual conectamos la señal TRIG del sensor
ECHOM = 24 #Variable que contiene el GPIO al cual conectamos la señal ECHO del sensor

TRIGR = 25
ECHOR = 5

TRIGL = 22
ECHOL = 27


SAMPLETIME = 0.5
TARGET = 33
KP = 0.05
KD = 0.025
KI = 0.00125

r = Robot((14,15), (4,3)) 


m1_speed = 1
m2_speed = 1
r.value = (m1_speed, m2_speed)

e1_prev_error = 0
e2_prev_error = 0

e1_sum_error = 0
e2_sum_error = 0
distanciaM=100
distanciaR=100
distanciaL=100

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIGM, GPIO.OUT) #Configuramos el pin TRIG como una salida 
GPIO.setup(ECHOM, GPIO.IN)  #Configuramos el pin ECHO como una salida 
GPIO.setup(TRIGR, GPIO.OUT)
GPIO.setup(ECHOR, GPIO.IN)
GPIO.setup(TRIGL, GPIO.OUT)
GPIO.setup(ECHOL, GPIO.IN)
GPIO.output(TRIGM, False)
GPIO.output(TRIGR, False)
GPIO.output(TRIGL, False)



def condicionParada(num):
    if 10 > num:
        print("Now stop")
        r.value = (0, 0)
        time.sleep(5)
            
            
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

#e1 = Encoder(17)
#e2 = Encoder(26)
e1 = DigitalInputDevice(17)
e2 = DigitalInputDevice(26)
def contador(count):
    count=count+1
    return count
while True:
    count=0
    i=0
    m1_max= 50
    m2_max= 50
    while i>=1000:
        i=i+1
        if e1.when_activated:
            count=contador(count)
    print("Contador ", count)
    e1_error = TARGET 
    e2_error = TARGET 
    print("Encoder I:",e1_error)
    print("Encoder D:", e2_error)
    m1_speed += (e1_error * KP) + (e1_prev_error * KD) + (e1_sum_error * KI)
    m2_speed += (e2_error * KP)  + (e1_prev_error * KD) + (e2_sum_error * KI)
    
    m1_speed = m1_speed/m1_max
    m2_speed = m2_speed/m2_max
    #m1_speed = max(min(1, m1_speed), 0)
    #m2_speed = max(min(1, m2_speed), 0)
    r.value = (m1_speed, m2_speed)

    
    print("m1 {} m2 {}".format(m1_speed, m2_speed))

    #e1.reset()
    #e2.reset()

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
