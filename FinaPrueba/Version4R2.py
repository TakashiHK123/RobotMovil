import concurrent.futures
import time
import math
import RPi.GPIO as GPIO
vSonido = 34300 #cm/s velocidad del sonido

TRIGM = 23 #Variable que contiene el GPIO al cual conectamos la señal TRIG del sensor
ECHOM = 24 #Variable que contiene el GPIO al cual conectamos la señal ECHO del sensor

TRIGR = 25
ECHOR = 5

TRIGL = 22
ECHOL = 20
ENCODER_LB = 17 #Encoder Izquierda PIN B
ENCODER_LA = 26 #Encoder Izquierda PIN A

ENCODER_RA = 16 #Encoder Derecha PIN A
ENCODER_RB = 21 #Encoder Derecha PIN B

SAMPLETIME = 0.5
TARGET = 25  #SetPoint
#Constantes de PID para cada motor
KPR = 1.05
KIR = 0.01
KDR = 0.001

KPL = 1.07
KIL = 0.01
KDL = 0.001


ALERTA=40
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
GPIO.setup(ENCODER_LA,GPIO.IN)  #Definimos los pin de encoders como entrada
GPIO.setup(ENCODER_LB,GPIO.IN)
GPIO.setup(ENCODER_RA,GPIO.IN)
GPIO.setup(ENCODER_RB,GPIO.IN)

GPIO.add_event_detect(ENCODER_LA, GPIO.FALLING)#Pin de encoder definidos para detectar eventos 
GPIO.add_event_detect(ENCODER_LB, GPIO.FALLING)
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
    if left==0:
        forwardL.stop() # stop PWM from GPIO output it is necessary
        reverseL.stop()
        GPIO.output(MotorE1,GPIO.LOW)
    else:
        forwardL.start(0)
        reverseL.start(0)
        GPIO.output(MotorE1,GPIO.HIGH)
        forwardL.ChangeDutyCycle(left)
        reverseL.ChangeDutyCycle(0)
    if right==0:
        forwardR.stop() # stop PWM from GPIO output it is necessary
        reverseR.stop()
        GPIO.output(MotorE2,GPIO.LOW)
    else:
        forwardR.start(0)
        reverseR.start(0)
        GPIO.output(MotorE2,GPIO.HIGH)
        forwardR.ChangeDutyCycle(right)
        reverseR.ChangeDutyCycle(0)

    
    
        
def convertidor(dato):
    print("Dato:     ", dato)
    dato=dato*100
    return abs(dato)

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
    pulsos = int((x*240)/20.73)
    cuentaRueda=0
    print("Pulsos:", pulsos)
    if grado<0:
        motorPwm(30,0)
        while True:
            #time.sleep(0.01)
            if GPIO.event_detected(ENCODER_LA):
                #print("Cantidad Contada L",cuentaRueda)
                cuentaRueda = cuentaRueda + 1
                if cuentaRueda==pulsos:
                    break
    if grado>0:
        motorPwm(0,30)
        while True:
            #time.sleep(0.01)
            if GPIO.event_detected(ENCODER_RA):
                #print("Cantidad Contada R",cuentaRueda)
                cuentaRueda = cuentaRueda + 1
                if cuentaRueda==pulsos:
                    break
    
    
def condicionParada(num):
    if 15 > num:
        print("Now stop")
        motorPwm(0,0)
        time.sleep(1)
        
def contador(count):
    count=count+1
    return count

def pruebaEncoder(pulsos, encoder):
    motorPwm(30,30)
    count=0
    while True:
        if GPIO.event_detected(encoder):
            count=contador(count)
            print("Se Detecto EncoderL")
        if count==pulsos:
            print("Se cumplio los: ",count," pulsos")
            break;
        
def countEncoder(encoder):
    count=0
    inicioPulso = time.time()
    try:
        while count<=30:
            if GPIO.event_detected(encoder):
                count=count+1
        finPulso = time.time()
        tiempo=abs(finPulso-inicioPulso)
        return tiempo
    except:
        return 0

def calcularVelocidad(tiempo):
    alfha=math.pi/4
    omega=alfha/tiempo
    velocidad=omega*3.3
    return velocidad
    
    
distanciaM=100
distanciaL=100
distanciaR=100
motorPwm(0,0)
time.sleep(3)
motorPwm(30,30)
m1Speed=0.5
m2Speed=0.5
while True:

    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
        method_thread = {
            executor.submit(countEncoder, ENCODER_LA):
            "process completed encoderL",
            executor.submit(countEncoder, ENCODER_RA):
            "process completed encoderR",
        }
    for future in concurrent.futures.as_completed(method_thread):
        etiqueta = method_thread[future]
        try:
            if etiqueta == "process completed encoderL":
                tiempoL=future.result()
            if etiqueta == "process completed encoderR":
                tiempoR=future.result()
        except Exception as exc:
            print('%r se a generado una exception: %s' % (final, exc))
                  
    
    velocidadL=calcularVelocidad(tiempoL)
    velocidadR=calcularVelocidad(tiempoR)
    print("VelocidadL:", velocidadL)
    print("VelocidadR:", velocidadR)
    
            
    e1Error = TARGET - velocidadL
    e2Error = TARGET - velocidadR
    print("---------VelocidadDiff-------",abs(velocidadL-velocidadR))
    print("Encoder I:",e1Error)
    print("Encoder D:", e2Error)
    m1Speed += (e1Error * KPL) + (e1PrevError * KDL) + (e1SumError * KIL)
    m2Speed += (e2Error * KPR)  + (e2PrevError * KDR) + (e2SumError * KIR)
    print("mLSpeed L",m1Speed)
    print("mRSpeed R",m2Speed)
    m1Speed = max(min(100, m1Speed), 20)
    m2Speed = max(min(100, m2Speed), 20)
    #m1Speed=convertidor(m1Speed)
    #m2Speed=convertidor(m2Speed)
    #print("valorOriginal L",m1Speed)
    #print("valorOriginal R",m2Speed)
    #print("---------PWMDiff-------",abs(m1Speed-m2Speed))
    motorPwm(m1Speed,m2Speed)
    #motorPwm(0,m2Speed)

    print("PWM L:", m1Speed)
    print("PWM R:", m2Speed)
    
    e1PrevError = e1Error
    e2PrevError = e2Error

    e1SumError += e1Error
    e2SumError += e2Error
    

                 
    distanciaM = medirDistancia(ECHOM, TRIGM,distanciaM)
    imprimirDistancia(distanciaM,"central")
    condicionParada(distanciaM)
    
    distanciaR = medirDistancia(ECHOR, TRIGR,distanciaR)
    imprimirDistancia(distanciaR,"derecha")
    condicionParada(distanciaR)
    
    distanciaL = medirDistancia(ECHOL, TRIGL,distanciaL)
    imprimirDistancia(distanciaL,"izquierda")
    condicionParada(distanciaL)
    motorPwm(m1Speed,m2Speed)

    anguloFR=campoPotencial(distanciaL, distanciaR, distanciaM)
    if distanciaL<ALERTA or distanciaR<ALERTA or distanciaM<ALERTA:
        print("Grado:", anguloFR)
        if anguloFR == None:
            print("Sigue Recto")
        elif anguloFR!=0 and abs(anguloFR)>8:
            print("AnguloFR:",anguloFR)
            controladorDriver(anguloFR)
            motorPwm(m1Speed,m2Speed)
    #Version Completa