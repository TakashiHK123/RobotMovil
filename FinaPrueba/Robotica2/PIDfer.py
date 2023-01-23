import threading
from gpiozero import DigitalInputDevice, Robot
from time import sleep
import time
import math
import RPi.GPIO as GPIO
import asyncio

v_sonido = 34300 #cm/s velocidad del sonido

TRIGM = 23 #Variable que contiene el GPIO al cual conectamos la señal TRIG del sensor
ECHOM = 24 #Variable que contiene el GPIO al cual conectamos la señal ECHO del sensor

TRIGR = 25
ECHOR = 5

TRIGL = 22
ECHOL = 27

ENCODERL = 17
ENCODERR = 26

MotorIN1 = 15
MotorIN2 = 14
MotorE1 = 13
MotorDIN1 = 3
MotorDIN2 = 4
MotorE2 = 12

SAMPLETIME = 0.02
TARGET = 45
KP = 2
KI = 0
KD = 0


#r = Robot((14,15), (4,3)) 


#m1_speed = 0.5
#m2_speed = 0.5
#r.value = (m1_speed, m2_speed)

e1_prev_error = 0
e2_prev_error = 0

e1_sum_error = 0
e2_sum_error = 0
distanciaM=100
distanciaR=100
distanciaL=100

GPIO.setmode(GPIO.BCM)     #Establecemos el modo según el cual nos refiriremos a los GPIO de nuestra RPi
GPIO.setup(MotorIN1,GPIO.OUT)
GPIO.setup(MotorIN2,GPIO.OUT)
GPIO.setup(MotorE1,GPIO.OUT)
GPIO.setup(MotorDIN1,GPIO.OUT)
GPIO.setup(MotorDIN2,GPIO.OUT)
GPIO.setup(MotorE2,GPIO.OUT)
GPIO.setup(TRIGM, GPIO.OUT) #Configuramos el pin TRIG como una salida 
GPIO.setup(ECHOM, GPIO.IN)  #Configuramos el pin ECHO como una salida 
GPIO.setup(TRIGR, GPIO.OUT)
GPIO.setup(ECHOR, GPIO.IN)
GPIO.setup(TRIGL, GPIO.OUT)
GPIO.setup(ECHOL, GPIO.IN)
GPIO.setup(ENCODERL,GPIO.IN)
GPIO.setup(ENCODERR,GPIO.IN)
GPIO.add_event_detect(ENCODERL, GPIO.FALLING)
GPIO.add_event_detect(ENCODERR, GPIO.FALLING)

GPIO.output(TRIGM, False)
GPIO.output(TRIGR, False)
GPIO.output(TRIGL, False)


print("Estabilizandose el sensor")
time.sleep(1)
print("Sensor estabilizado")

forward=GPIO.PWM(MotorIN2,100) # configuring Enable pin for PWM
reverse=GPIO.PWM(MotorIN1,100) # configuring Enable pin for PWM
forward2=GPIO.PWM(MotorDIN2,100) # configuring Enable pin for PWM
reverse2=GPIO.PWM(MotorDIN1,100)



def condicionParada(num):
    if 10 > num:
        print("Now stop")
        forward.ChangeDutyCycle(0)
        reverse.ChangeDutyCycle(0)
        forward2.ChangeDutyCycle(0)
        reverse2.ChangeDutyCycle(0)
        time.sleep(5)
            
            
def medirDistancia(ECHO, TRIG, distanciaAnterior):
    GPIO.output(TRIG, True) #Trig sensor medio alto
    time.sleep(0.00001) #Delay de 100 microsegundos
    GPIO.output(TRIG, False) #Trig sensor medio bajo
    try:
        while GPIO.input(ECHO)==0:
            inicioPulso = time.time()
            #print("Inicio pulso: ", inicio_pulso)
        while GPIO.input(ECHO)==1:
            finPulso = time.time()
            #print("Fin pulso: ", fin_pulso)
        tiempo = finPulso - inicioPulso
        distancia = vSonido * (tiempo/2)
        distancia = round(distancia,2)
        
        if distancia > 2 and distancia < 400:
            return distancia
        else:
            return distanciaAnterior
    except:
        return distanciaAnterior
        

def imprimirDistancia(result, sensor):
    print("Distancia ",sensor,": ",result, " cm")
    
def contador(count):
    count=count+1
    return count

class PID_RASPY:
    def __init__(self, setpoint,Sample_time, KD,KP,KI):
        self.Setpoint= setpoint
        self.KD= KD
        self.KP= KP
        self.KI= KI
        self.Sample_time= Sample_time
        self.last_Time=0
        self.I_term=0
        self.Out_max=100
        self.Out_min=20
        self.last_input=0
        self.output=0
        
    def calcular_esfuerzo(self,entrada):
        lista=[]
        tiempo_actual=time.time()
        variacion_tiempo= tiempo_actual - self.last_Time
        if(variacion_tiempo >= self.Sample_time):
            print("VariacionTemp:",variacion_tiempo)
            #calculamos los errores
            error= self.Setpoint - entrada
            self.I_term += self.KI* error
            
            #aqui se regula la acumalacion de los terminos integrales
            if self.I_term > self.Out_max :
                self.I_term = self.Out_max
            
            if self.I_term < self.Out_min:
                self.I_term = self.Out_min
               
            
            dInput = entrada - self.last_input
        
            #calculamos la salida del PID
            self.output= self.KP*error + self.I_term - self.KD*dInput
            #self.output= self.KP*error 
            #aqui se regula la salida
            if self.output > self.Out_max :
                self.output=self.Out_max
            
            if self.output < self.Out_min :
                self.output=self.Out_min
        
            #guardamos las variables para la proxima iteracion
            self.last_input = entrada
            self.last_Time  = tiempo_actual
        
    
def getTime(pulsosPoint):
    cuentaRuedaR=0
    cuentaRuedaL=0
    variacion=[0.0,0.0]
    variacionR=0
    variacionL=0
    tiempoActualR=time.time()
    tiempoActualL=time.time()
    while True:
        if GPIO.event_detected(ENCODERR):
            cuentaRuedaR = cuentaRuedaR + 1
            if cuentaRuedaR==pulsosPoint:
                tiempoFinalR=time.time()
                variacionR=tiempoFinalR-tiempoActualR
                print("CuentaPulso:",cuentaRuedaR)
                variacion[0]=variacionR
        if GPIO.event_detected(ENCODERL):
            cuentaRuedaL = cuentaRuedaL + 1
            if cuentaRuedaL==pulsosPoint:
                tiempoFinalL=time.time()
                variacionL=tiempoFinalL-tiempoActualL
                print("CuentaPulso:",cuentaRuedaL)
                variacion[1]=variacionL
        if cuentaRuedaL==pulsosPoint and cuentaRuedaR==pulsosPoint:
            return variacion
        
def buildVelocidad(tiempo):
    velocidad=(math.pi*6.6)/tiempo
    return velocidad

def pwmMotorForward(motorL,motorR):
    
    forward.ChangeDutyCycle(motorL)
    reverse.ChangeDutyCycle(0)
    forward2.ChangeDutyCycle(motorR)
    reverse2.ChangeDutyCycle(0)
    
def pwmMotorReverse(motorL,motorR):
    forward.ChangeDutyCycle(0)
    reverse.ChangeDutyCycle(motorL)
    forward2.ChangeDutyCycle(0)
    reverse2.ChangeDutyCycle(motorR)


GPIO.output(MotorE1,GPIO.HIGH)
GPIO.output(MotorE2,GPIO.HIGH)    
forward.start(0) 
reverse.start(0)
forward2.start(0) 
reverse2.start(0)
pwmMotorForward(50,50)

distanciaM=100
distanciaR=100
distanciaL=100
#pwmMotorForward(100,100)
motor1=PID_RASPY(TARGET,SAMPLETIME,KD,KP,KI)
motor2=PID_RASPY(TARGET,SAMPLETIME,KD,KP,KI)
while True:
    #motor1=PID_RASPY(TARGET,SAMPLETIME,KD,KP,KI)
    #motor2=PID_RASPY(TARGET,SAMPLETIME,KD,KP,KI)
    tiempo=getTime(38)
    tiempo1=tiempo[0]
    tiempo2=tiempo[1]
    print("Tiempo1:",tiempo1)
    print("Tiempo2:",tiempo2)
    vel1=buildVelocidad(tiempo1)
    vel2=buildVelocidad(tiempo2)
    print("Velocidad_Izquierda:",vel1)
    print("Velocidad_Derecha:",vel2)
    #motor2=PID_RASPY(TARGET,1,KD,KP,KI)
    motor1.calcular_esfuerzo(vel1)
    motor2.calcular_esfuerzo(vel2)
    velocidad1=motor1.output
    velocidad2=motor2.output
    print("PWM_PID_Izquierda:", velocidad1)
    print("PWM_PID_Derecha:", velocidad2)
    #print("I_item:",motor1.I_term)
    #print("last_time:",motor1.last_Time)
    #r.value = (m1_speed, m2_speed)
    #r.value = (velocidad1, velocidad2)
    pwmMotorForward(velocidad1,velocidad2)

    distanciaM = medirDistancia(ECHOM, TRIGM,distanciaM)
    imprimirDistancia(distanciaM,"central")
    condicionParada(distanciaM)

    distanciaR = medirDistancia(ECHOR, TRIGR,distanciaR)
    imprimirDistancia(distanciaR,"derecha")
    condicionParada(distanciaR)

    distanciaL = medirDistancia(ECHOL, TRIGL,distanciaL)
    imprimirDistancia(distanciaL,"izquierda")
    condicionParada(distanciaL)
