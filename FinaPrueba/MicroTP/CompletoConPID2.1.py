import threading
from gpiozero import DigitalInputDevice, Robot
from time import sleep
import time
import math
import RPi.GPIO as GPIO


v_sonido = 34300 #cm/s velocidad del sonido

TRIGM = 23 #Variable que contiene el GPIO al cual conectamos la señal TRIG del sensor
ECHOM = 24 #Variable que contiene el GPIO al cual conectamos la señal ECHO del sensor

TRIGR = 25
ECHOR = 5

TRIGL = 22
ECHOL = 27
ENCODERL = 17
ENCODERR = 26

SAMPLETIME = 0.5
TARGET = 15
KP = 0.02
KI = 0.005
KD = 0.01
ALERTA=60
r = Robot((14,15), (4,3)) 


m1_speed = 1
m2_speed = 0.85
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
GPIO.setup(ENCODERL,GPIO.IN)
GPIO.setup(ENCODERR,GPIO.IN)
#GPIO.setup(ENCODERL, GPIO.IN, pull_up_down=GPIO.PUD_UP)
#GPIO.setup(ENCODERR, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(ENCODERL, GPIO.FALLING)
GPIO.add_event_detect(ENCODERR, GPIO.FALLING)
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
        
def campoPotencial(distanciaIzquierda, distanciaDerecha, distanciaCentral):
    angulo=0
    fx=distanciaIzquierda*math.cos((33*math.pi)/180) - distanciaDerecha*math.cos((33*math.pi)/180)
    fy=200-distanciaCentral-distanciaIzquierda*math.sin((33*math.pi)/180) - distanciaDerecha*math.sin((33*math.pi)/180)
    if fx>0 and fy>0:
        angulo= -(math.atan(abs(fx/fy)))
        angulo=(angulo*180)/math.pi
        return angulo
    elif fx<0 and fy>0:
        angulo= (math.atan(abs(fx/fy)))
        angulo=(angulo*180)/math.pi
        return angulo
    #elif fx<0 and fy<0:
        #angulo = 180 - math.atan(abs(fx/fy))
        #angulo=(angulo*180)/math.pi
        #return angulo
    #elif fy<0 and fx>0:
        #angulo = math.atan(abs(fx/fy)) - 180
        #angulo=(angulo*180)/math.pi
        #return angulo

def imprimirDistancia(result, sensor):
    print("Distancia ",sensor,": ",result, " cm")
    
def controladorDriver(grado):
    x=((abs(grado)*math.pi)*13)/180
    pulsos = int((x*20)/20.73)
    cuentaRueda=0
    #print("Pulsos:", pulsos)
    if grado<0 and abs(grado)<90:
        r.value = (0, 1)
        while True:
            time.sleep(0.01)
            if GPIO.event_detected(ENCODERR):
                #print("Cantidad Contada L",cuentaRueda)
                cuentaRueda = cuentaRueda + 1
                if cuentaRueda>pulsos:
                    break
    if grado>0 and grado<90:
        r.value = (1, 0)
        while True:
            time.sleep(0.01)
            if GPIO.event_detected(ENCODERL):
                #print("Cantidad Contada R",cuentaRueda)
                cuentaRueda = cuentaRueda + 1
                if cuentaRueda>pulsos:
                    break
    #if grado<0 and abs(grado)>90:
        #r.value = (0.5, 0)
        #while True:
            #time.sleep(0.01)
            #if GPIO.event_detected(ENCODERL):
                #print("Cantidad Contada R",cuentaRueda)
                #cuentaRueda = cuentaRueda + 1
                #if cuentaRueda>pulsos:
                    #r.value = (0, 0)
                    #time.sleep(0.01)
                    #break
    #if grado>0 and abs(grado)>90:
        #r.value = (0, 0.5)
        #while True:
            #time.sleep(0.01)
            #if GPIO.event_detected(ENCODERR):
                #print("Cantidad Contada R",cuentaRueda)
                #cuentaRueda = cuentaRueda + 1
                #if cuentaRueda>pulsos:
                    #r.value = (0, 0)
                    #time.sleep(0.01)
                    #break
    
def contador(count):
    count=count+1
    return count

while True:
    i=0
    countL=0
    countR=0
    while i<=150000:
        i=i+1
        if GPIO.event_detected(ENCODERL):
            countL=contador(countL)
        if GPIO.event_detected(ENCODERR):
            countR=contador(countR)
    print("ContadorL:",countL)
    print("ContadorR:",countR)
    
    e1_error = TARGET - countL
    e2_error = TARGET - countR
    #print("Encoder I:",e1_error)
    #print("Encoder D:", e2_error)
    m1_speed += (e1_error * KP) + (e1_prev_error * KD) + (e1_sum_error * KI)
    m2_speed += (e2_error * KP)  + (e1_prev_error * KD) + (e2_sum_error * KI)
    #print("VelocidadL:", m1_speed)
    #print("VelocidadR:", m2_speed)
    m1_speed = max(min(1, m1_speed), 0)
    m2_speed = max(min(1, m2_speed), 0)
    r.value = (m1_speed, 0.85)
    #r.value = (0, 0.5)

    #print("m1 {} m2 {}".format(m1_speed, m2_speed))

    #e1.reset()
    #e2.reset()

    #sleep(SAMPLETIME)

    e1_prev_error = e1_error
    e2_prev_error = e2_error

    e1_sum_error += e1_error
    e2_sum_error += e2_error
    
    distanciaM = medirDistancia(ECHOM, TRIGM,distanciaM)
    imprimirDistancia(distanciaM,"central")
    condicionParada(distanciaM)
    
    distanciaR = medirDistancia(ECHOR, TRIGR,distanciaR)
    imprimirDistancia(distanciaR,"derecha")
    condicionParada(distanciaR)
    
    distanciaL = medirDistancia(ECHOL, TRIGL,distanciaL)
    imprimirDistancia(distanciaL,"izquierda")
    condicionParada(distanciaL)
    
    anguloFR=campoPotencial(distanciaL, distanciaR, distanciaM)
    if distanciaL<ALERTA or distanciaR<ALERTA or distanciaM<ALERTA:
        #print("Grado:", anguloFR)
        if anguloFR == None:
            print("Sigue Recto")
        elif anguloFR!=0 and abs(anguloFR)>8:
            print("AnguloFR:",anguloFR)
            controladorDriver(anguloFR)
    r.value = (m1_speed, 0.85)
    #r.value = (1, 0)
