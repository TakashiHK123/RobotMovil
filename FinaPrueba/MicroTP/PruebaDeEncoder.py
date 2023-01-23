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
ECHOL = 20

ENCODER_LB = 17 #Encoder Izquierda pin B
ENCODER_LA = 26 #Encoder Izquierda pin A
ENCODER_RA = 16	#Encoder Derecha pin A
ENCODER_RB = 21 #Encoder Derecha pin B

SAMPLETIME = 0.5

TARGET = 10  #SetPoint
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


e1_prev_error = 0
e2_prev_error = 0

e1_sum_error = 0
e2_sum_error = 0

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIGM, GPIO.OUT) #Configuramos el pin TRIG como una salida 
GPIO.setup(ECHOM, GPIO.IN)  #Configuramos el pin ECHO como una salida 
GPIO.setup(TRIGR, GPIO.OUT)
GPIO.setup(ECHOR, GPIO.IN)
GPIO.setup(TRIGL, GPIO.OUT)
GPIO.setup(ECHOL, GPIO.IN)
GPIO.setup(ENCODER_LB,GPIO.IN)  #Definimos los pin de encoders como entrada 
GPIO.setup(ENCODER_LA,GPIO.IN)
GPIO.setup(ENCODER_RA,GPIO.IN)
GPIO.setup(ENCODER_RB,GPIO.IN)

GPIO.add_event_detect(ENCODER_LB, GPIO.FALLING)#Pin de encoder definidos para detectar eventos 
GPIO.add_event_detect(ENCODER_LA, GPIO.FALLING)
GPIO.add_event_detect(ENCODER_RA, GPIO.FALLING)
GPIO.add_event_detect(ENCODER_RB, GPIO.FALLING)

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


def motorPwm(left,right):
    if left==0 and right==0:
        forwardL.ChangeDutyCycle(left)
        reverseL.ChangeDutyCycle(0)
        forwardR.ChangeDutyCycle(right)
        reverseR.ChangeDutyCycle(0)
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
    dato=dato*100
    return dato

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
        
def campoPotencial(distanciaIzquierda, distanciaDerecha, distanciaCentral): #Campo Potencial, solo para direcciones frontales
    angulo=0
    distanciaIzquierda=30/distanciaIzquierda
    distanciaDerecha=30/distanciaDerecha
    distanciaCentral=30/distanciaCentral
    fx=distanciaIzquierda*math.cos((33*math.pi)/180) - distanciaDerecha*math.cos((33*math.pi)/180)
    fy=1-distanciaCentral-distanciaIzquierda*math.sin((33*math.pi)/180) - distanciaDerecha*math.sin((33*math.pi)/180)
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
            time.sleep(0.01)
            if GPIO.event_detected(ENCODERL):
                #print("Cantidad Contada L",cuentaRueda)
                cuentaRueda = cuentaRueda + 1
                if cuentaRueda>pulsos:
                    break
    if grado>0:
        motorPwm(0,30)
        while True:
            time.sleep(0.01)
            if GPIO.event_detected(ENCODERR):
                #print("Cantidad Contada R",cuentaRueda)
                cuentaRueda = cuentaRueda + 1
                if cuentaRueda>pulsos:
                    break
    
    
def condicionParada(num):
    if 10 > num:
        print("Now stop")
        motorPwm(0,0)
        time.sleep(2)
        
def contador(count):
    count=count+1
    return count

def pruebaEncoder(pulsos, encoder):
    if encoder == 17:
        mensaje="LB"
    if encoder == 26:
        mensaje="LA"
    if encoder == 16:
        mensaje="RA"
    if encoder == 21:
        mensaje ="RB"
    count=0
    while True:
        if GPIO.event_detected(encoder):
            count=contador(count)
            print("Se Detecto Encoder: ", mensaje)
            print(count)
        if count==pulsos:
            print("Se cumplio los: ",count," pulsos")
            motorPwm(0,0)
            break;
            
        


motorPwm(0,50)
pruebaEncoder(240,ENCODER_RA)


motorPwm(50,0)
pruebaEncoder(240,ENCODER_LA)
