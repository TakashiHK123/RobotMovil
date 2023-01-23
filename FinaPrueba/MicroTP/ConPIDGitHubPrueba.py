import threading
import time
import math
from gpiozero import DigitalInputDevice
import RPi.GPIO as GPIO
from Encoder import Encoder
vSonido = 34300 #cm/s velocidad del sonido

TRIGM = 23 #Variable que contiene el GPIO al cual conectamos la señal TRIG del sensor
ECHOM = 24 #Variable que contiene el GPIO al cual conectamos la señal ECHO del sensor

TRIGR = 25
ECHOR = 5

TRIGL = 22
ECHOL = 27
ENCODERL = 17 #Encoder Izquierda
ENCODERR = 26 #Encoder Derecha

SAMPLETIME = 0.5
TARGET = 15  #SetPoint
KP = 0.02
KI = 0.005
KD = 0.01
ALERTA=45
#Definimos los pines para los motores para las direcciones 
#Configuracion del motor GPIO
MotorIN1 = 15
MotorIN2 = 14
MotorE1 = 13
MotorDIN1 = 3
MotorDIN2 = 4
MotorE2 = 12


e1PrevError = 0
e2PrevError = 0

e1SumError = 0
e2SumError = 0

inicioPulso=0
finPulso=0
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIGM, GPIO.OUT) #Configuramos el pin TRIG como una salida 
GPIO.setup(ECHOM, GPIO.IN)  #Configuramos el pin ECHO como una salida 
GPIO.setup(TRIGR, GPIO.OUT)
GPIO.setup(ECHOR, GPIO.IN)
GPIO.setup(TRIGL, GPIO.OUT)
GPIO.setup(ECHOL, GPIO.IN)
#GPIO.setup(ENCODERL,GPIO.IN)  #Definimos los pin de encoders como entrada 
#GPIO.setup(ENCODERR,GPIO.IN)
#GPIO.add_event_detect(ENCODERL, GPIO.FALLING)#Pin de encoder definidos para detectar eventos 
#GPIO.add_event_detect(ENCODERR, GPIO.FALLING)
GPIO.output(TRIGM, False)#para limpiar la salida de los ultrasonidos 
GPIO.output(TRIGR, False)
GPIO.output(TRIGL, False)

GPIO.setup(MotorIN1,GPIO.OUT)
GPIO.setup(MotorIN2,GPIO.OUT)
GPIO.setup(MotorE1,GPIO.OUT)
GPIO.setup(MotorDIN1,GPIO.OUT)
GPIO.setup(MotorDIN2,GPIO.OUT)
GPIO.setup(MotorE2,GPIO.OUT)


forwardL=GPIO.PWM(MotorIN2,100) # configuring Enable pin for PWM
reverseL=GPIO.PWM(MotorIN1,100) # configuring Enable pin for PWM
forwardR=GPIO.PWM(MotorDIN2,100) # configuring Enable pin for PWM
reverseR=GPIO.PWM(MotorDIN1,100)
            
print("Estabilizando los motores")
time.sleep(1)
print("Motores estables")

'''class Encoder(object):
    def __init__(self,pin):
        self._value=0
        encoder = DigitalInputDevice(pin)
        encoder.when_activated = self._increment
        encoder.when_deactivated = self._increment
    def reset(self):
        self._value =0
    def _increment(self):
        self._value = self._value + 1

    @property
    def value(self):
        return self._value'''
    
def motorPwm(left,right):
    if left==0 and right==0:
        forwardL.stop() # stop PWM from GPIO output it is necessary
        reverseL.stop()
        forwardR.stop() # stop PWM from GPIO output it is necessary
        reverseR.stop()
        
    forwardL.start(0) 
    reverseL.start(0)
    forwardR.start(0) 
    reverseR.start(0)
    GPIO.output(MotorE1,GPIO.HIGH)
    GPIO.output(MotorE2,GPIO.HIGH)  
    forwardL.ChangeDutyCycle(left)
    reverseL.ChangeDutyCycle(0)
    forwardR.ChangeDutyCycle(right)
    reverseR.ChangeDutyCycle(0)
    
        
def convertidor(dato):
    print("Dato:     ", dato)
    dato=dato*100
    return dato

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
    
def campoPotencial(distanciaIzquierda, distanciaDerecha, distanciaCentral): #Campo Potencial, solo para direcciones frontales
    angulo=0
    distanciaIzquierda=distanciaIzquierda
    distanciaDerecha=distanciaDerecha
    distanciaCentral=distanciaCentral
    fx=distanciaIzquierda*math.cos((33*math.pi)/180) - distanciaDerecha*math.cos((33*math.pi)/180)
    fy=400-distanciaCentral-distanciaIzquierda*math.sin((33*math.pi)/180) - distanciaDerecha*math.sin((33*math.pi)/180)
    if fx>0 and fy>0:
        angulo= -(math.atan(abs(fx/fy)))
        angulo=(angulo*180)/math.pi
        print("1erCuadrante: ",angulo)
        return angulo
    elif fx<0 and fy>0:
        angulo= (math.atan(abs(fx/fy)))
        angulo=(angulo*180)/math.pi
        print("2erCuadrante: ",angulo)
        return angulo
    elif fx<0 and fy<0:
        angulo = math.pi - math.atan(abs(fx/fy))
        angulo=(angulo*180)/math.pi
        print("3erCuadrante: ",angulo)
        return angulo
    elif fy<0 and fx>0:
        angulo = math.atan(abs(fx/fy)) - math.pi
        angulo=(angulo*180)/math.pi
        print("4toCuadrante: ",angulo)
        return angulo

def imprimirDistancia(result, sensor):
    print("Distancia ",sensor,": ",result, " cm")
    
def controladorDriver(grado):  #Para girar los grados necesarios que le enviamos por el campo potencial
    grado=grado
    x=((abs(grado)*math.pi)*13)/180
    pulsos = int((x*20)/20.73)
    cuentaRueda=0
    print("Pulsos:", pulsos)
    if grado<0:
        motorPwm(30,0)
        while True:
            #time.sleep(0.01)
            if GPIO.event_detected(ENCODERL):
                #print("Cantidad Contada L",cuentaRueda)
                cuentaRueda = cuentaRueda + 1
                if cuentaRueda>pulsos:
                    break
    if grado>0:
        motorPwm(0,30)
        while True:
            #time.sleep(0.01)
            if GPIO.event_detected(ENCODERR):
                #print("Cantidad Contada R",cuentaRueda)
                cuentaRueda = cuentaRueda + 1
                if cuentaRueda>pulsos:
                    break
    
    
def condicionParada(num):
    if 10 > num:
        print("Now stop")
        motorPwm(0,0)
        #time.sleep(2)
        
def contador(count):
    count=count+1
    return count

def pruebaEncoder(pulsos, encoder):
    motorPwm(30,30)
    count=0
    e1=Encoder(encoder)
    while True:
        #if GPIO.event_detected(encoder):
            #count=contador(count)
            #print("Se Detecto EncoderL")
        #if count==pulsos:
            #print("Se cumplio los: ",count," pulsos")
            #break;
        print("-")
        if e1.value==pulsos:
            print("Se cumplio los :", e1.value," pulsos")
            break
        

        
        
distanciaM=100
distanciaL=100
distanciaR=100
m1Speed=0.5
m2Speed=0.5
motorPwm(100,100)
encoderL = Encoder(17)
encoderR = Encoder(26)
while True:
    #i=0
    #countL=0
    #countR=0
    #while i<=180000:
        #i=i+1
        #if GPIO.event_detected(ENCODERL):
            #countL=contador(countL)
        #if GPIO.event_detected(ENCODERR):
            #countR=contador(countR)
    #print("ContadorL:",countL)
    #print("ContadorR:",countR)
    
    #e1Error = TARGET - countL
    #e2Error = TARGET - countR
    e1Error = TARGET - encoderL.value
    e2Error = TARGET - encoderR.value
    print("EncoderL.Value(): ",encoderL.value)
    print("EncoderR.Value(): ",encoderR.value)

    #print("Encoder I:",e1Error)
    #print("Encoder D:", e2Error)
    m1Speed += (e1Error * KP) + (e1PrevError * KD) + (e1SumError * KI)
    m2Speed += (e2Error * KP)  + (e2PrevError * KD) + (e2SumError * KI)
    print("VelocidadL:", m1Speed)
    print("VelocidadR:", m2Speed)
    m1Speed = max(min(100, m1Speed), 30)
    m2Speed = max(min(100, m2Speed), 30)
    #m1Speed=convertidor(m1Speed)
    #m2Speed=convertidor(m2Speed)
    motorPwm(m1Speed,m2Speed)
    
    print("PWM L:", m1Speed)
    print("PWM R:", m2Speed)

    print("encoderL {} encoderR{}".format(encoderL.value, encoderR.value))

    encoderL.reset()
    encoderR.reset()
 
    time.sleep(1)
    
    e1PrevError = e1Error
    e2PrevError = e2Error

    e1SumError += e1Error
    e2SumError += e2Error
    
    
    #distanciaM = medirDistancia(ECHOM, TRIGM,distanciaM)
    #imprimirDistancia(distanciaM,"central")
    #condicionParada(distanciaM)
    
    #distanciaR = medirDistancia(ECHOR, TRIGR,distanciaR)
    #imprimirDistancia(distanciaR,"derecha")
    #condicionParada(distanciaR)
    
    #distanciaL = medirDistancia(ECHOL, TRIGL,distanciaL)
    #imprimirDistancia(distanciaL,"izquierda")
    #condicionParada(distanciaL)
    
    #anguloFR=campoPotencial(distanciaL, distanciaR, distanciaM)
    #if distanciaL<ALERTA or distanciaR<ALERTA or distanciaM<ALERTA:
        #print("Grado:", anguloFR)
        #if anguloFR == None:
            #print("Sigue Recto")
        #elif anguloFR!=0 and abs(anguloFR)>8:
            #print("AnguloFR:",anguloFR)
            #controladorDriver(anguloFR)
    #Version Completa