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
TARGET = 30  #SetPoint
#Constantes de PID para cada motor
KPR = 1.05
KIR = 0.07
KDR = 0.0

KPL = 1.05
KIL = 0.01
KDL = 0.0

#Valores de PID para controlara la direccion
KPL2 = 2.8
KIL2 = 0.01
KDL2 = 0.0

KPR2 = 2.5
KIR2 = 0.09
KDR2 = 0.0

ALERTA=40
#Definimos los pines para los motores para las direcciones 
#Configuracion del motor GPIO
MotorIN1 = 15
MotorIN2 = 14
MotorE1 = 13
MotorDIN1 = 3
MotorDIN2 = 4
MotorE2 = 12

#Configuracion de las constantes para el campo potencial 
Katrac=10 #factor positivo de atraccion 
objetivoDistancia= 200 #distancia objetivo
Krepul = 5

#Congiguracion iniciales del PID 
e1PrevError = 0
e2PrevError = 0

e1SumError = 0
e2SumError = 0

#Inicializador de variables para nuestros sensor de distancia 
inicioPulso=0
finPulso=0

#Valores para controlar movimiento
R=3.3 #Radio de la rueda 3.3cm
b=9 #Distancia del punto de contacto de la rueda al punto medio del eje

#Valores para La fuerza de atraccion
ed = 0.003 #constante ajustable
xd=0
yd=200

#Valores del punto descentrado
e=13 #13cm del centro del eje de giro
alfha=math.pi/2 #90grados en radianes consideramos siempre nuestro eje en el centro del robot 

#Valores de repulsion
er=0.0028 #Constante ajustable
Qmin=1
Qinf=45

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
    
    
def condicionParada(num):
    if 8 > num:
        print("Now stop")
        motorPwm(0,0)
        time.sleep(1)
        return True
        
def contador(count):
    count=count+1
    return count

        
def countEncoder(encoder,timeAnterior):  
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
        return timeAnterior
    

def calcularVelocidadLineal(tiempo):
    alfha=(45*math.pi*2)/360
    omega=alfha/tiempo
    velocidad=omega*R #Radio de la rueda 3.3 cm 
    return velocidad

def calcularVelocidadAngular(tiempo):
    alfha=(45*math.pi*2)/360  #Calcula la velocidad cada 15 pulsos de los 240 por vuelta
    omega=alfha/tiempo
    return omega
    
def velocidadAngular(wl,wr):
    w=(R*(wr-wl))*2*b
    return w

def fuerzaDeAtraccion(dd): #nuestra fuera de atraccion es en linea recta a una distancia fija en y 
    Fatrac=ed*dd
    return Fatrac
    
def gradienteDeRepulsion(l,distanciaObstaculo):  #La l es la distancia con respecto al eje x o y
    if distanciaObstaculo>=Qmin and distanciaObstaculo<=Qinf:  #Condicion de proximidad 
        gradienteRepulsion=(er*l)/(distanciaObstaculo**3)
    else:
        gradienteRepulsion=0
    return gradienteRepulsion

def fuerzaDeRepulsion(gradienteRepulsion):
    Frepul=(-gradienteRepulsion)
    return Frepul
    
def velocidadCentroRobot(wl,wr):
    v=(R*(wl+wr))/2
    return v

def velocidadPuntoDescentradoY(w, alfha, vy): #w velocidad angular, alfha angulo a girar v velocidad lineal en y
    velDescY= vy + e*w*math.cos(alfha)
    if alfha==math.pi/2: #Hacemos esta condicion porque python no tira un cero al darle cos(90), tira un valor aproximado al cero 
        velDescY=vy
    return velDescY

def velocidadPuntoDescentradoX(w, alfha, vx):
    velDescX= vx - e*w*math.sin(alfha)
    if alfha==math.pi: #Condicionamos ya que el valor del pi no es exacto
        velDescX=vx
    return velDescX

def velocidadCentroX(v,alfha):
    vx=v*math.cos(alfha)
    if alfha==math.pi/2:
        vx=0
    return vx

def velocidadCentroY(v,alfha):
    vy=v*math.sin(alfha)
    if alfha==math.pi:
        vy=0
    return vy

def fuerzaX(FatracX,FrepX,FrepY,alpha):
    fx=FatracX + FrepX*math.cos(alpha) - FrepY*math.sin(alpha)
    return fx

def fuerzaY(FatracY,FerpX,FrepY,alpha):
    fy=FatracY + FrepX*math.sin(alpha) + FrepY*math.cos(alpha)
    return fy

def calculoLxL(distancia): #Calcula la distancia en el eje x del sensor Izquierda
    lx = (-distancia*math.sin((33*math.pi)/180))
    return lx

def calculoLxR(distancia):
    lx = distancia*math.sin((33*math.pi)/180)
    return lx

def calculoLy(distancia):
    ly = distancia*math.cos((33*math.pi)/180)#Solo calculamos uno ya que es del mismo signo en y los dos sensores del costado
    return ly


distanciaM=100
distanciaL=100
distanciaR=100
motorPwm(0,0)
time.sleep(3)
motorPwm(40,40)
m1Speed=0.5
m2Speed=0.5
alpha=math.pi/2
tiempoL=0
tiempoR=0
phi=math.pi/2
w=0
estado="derecho"
while True:
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
        method_thread = {
            executor.submit(countEncoder, ENCODER_LA,tiempoL):
            "process completed encoderL",
            executor.submit(countEncoder, ENCODER_RA,tiempoR):
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
   
    
    vl=calcularVelocidadLineal(tiempoL)
    vr=calcularVelocidadLineal(tiempoR)
    print("VelocidadL:", vl)
    print("VelocidadR:", vr)
    
    wl=calcularVelocidadAngular(tiempoL)
    wr=calcularVelocidadAngular(tiempoR)
    
    e1Error = TARGET - vl
    e2Error = TARGET - vr

    #print("error=",e2Error-e1Error)

    m1Speed += (e1Error * KPL) + (e1PrevError * KDL) + (e1SumError * KIL)
    m2Speed += (e2Error * KPR)  + (e2PrevError * KDR) + (e2SumError * KIR)

    m1Speed = max(min(80, m1Speed), 20)
    m2Speed = max(min(80, m2Speed), 20)


    #print("mL:",m1Speed)
    #print("mR:",m2Speed)
    print("Derecho")

    motorPwm(m1Speed,m2Speed)
    e1PrevError = e1Error
    e2PrevError = e2Error

    e1SumError += e1Error
    e2SumError += e2Error
    
    
    distanciaM = medirDistancia(ECHOM, TRIGM,distanciaM)
    #imprimirDistancia(distanciaM,"central")
    
    
    
    distanciaR = medirDistancia(ECHOR, TRIGR,distanciaR)
    #imprimirDistancia(distanciaR,"derecha")
    
    distanciaL = medirDistancia(ECHOL, TRIGL,distanciaL)
    #imprimirDistancia(distanciaL,"izquierda")
  

    #Calculamos las gradientes de repulsion y seran 3 por los tres sensores que son 3 obstaculos detectables por el robot
    
    if distanciaM<=Qinf or distanciaL<=Qinf or distanciaR<=Qinf:
        if estado=='derecho':
            e1PrevError=0
            e2PrevError=0
            e1SumError=0
            e2SumError=0
            estado='girando'
        gradienteRepulsionXM = gradienteDeRepulsion(0,distanciaM) #Para el sensor del medio lx es cero
        gradienteRepulsionYM = gradienteDeRepulsion(distanciaM,distanciaM)
        
        lxl=calculoLxL(distanciaL)
        gradienteRepulsionXL = gradienteDeRepulsion(lxl,distanciaL)
        lyl=calculoLy(distanciaL)
        gradienteRepulsionYL = gradienteDeRepulsion(lyl,distanciaL)
        
        lxR=calculoLxR(distanciaR)
        gradienteRepulsionXR = gradienteDeRepulsion(lxR,distanciaR)
        lyR=calculoLy(distanciaR)
        gradienteRepulsionYR = gradienteDeRepulsion(lyR,distanciaR)
        
        #Sumatoria de los gradiente de repulsion 
        deltaRepulsionX=gradienteRepulsionXM+gradienteRepulsionXL+gradienteRepulsionXR
        deltaRepulsionY=gradienteRepulsionYM+gradienteRepulsionYL+gradienteRepulsionYR
        
        FrepX = fuerzaDeRepulsion(deltaRepulsionX)
        FrepY = fuerzaDeRepulsion(deltaRepulsionY)
        print("---------------------------------------FuerzaRepX: ",FrepX)
        #print("FuerzaRepY: ",FrepY)
        #Calculamos la fuerza de Atraccion fija en un punto constante con respecto al robot en linea recta a una distancia fiaja 
        FatracX = fuerzaDeAtraccion(0)
        FatracY = fuerzaDeAtraccion(200)
        
        fx=fuerzaX(FatracX,FrepX,FrepY,alfha)
        fy=fuerzaY(FatracY,FrepX,FrepY,alfha)
        print("-----------------------------------------------------------------------------------------------Fx:",fx)
        #Aplicamos el descenso por gradiente para sacarl las velocidades angulares para girar 
        w=velocidadAngular(wl,wr)
        v=velocidadCentroRobot(wl,wr)
        
        vx=velocidadCentroX(v,alfha)
        vy=velocidadCentroY(v,alfha)
        vRefX=velocidadPuntoDescentradoX(w, alfha, vx)
        vRefY=velocidadPuntoDescentradoY(w, alfha, vy)
        nRefX=vRefX*fx+vRefY*fy
        nRefY=vRefY*fx+vRefY*fy
        
        
        #Calculamos las velocidades angulares de los motores a utilizar
        
        wl = (1/R)*(nRefX*(math.cos(alfha) + (b/e)*math.sin(alfha)) + nRefY*(math.sin(alfha) - (b/e)*math.cos(alfha)))
        
        wr = (1/R)*(nRefX*(math.cos(alfha) - (b/e)*math.sin(alfha)) + nRefY*(math.sin(alfha) + (b/e)*math.cos(alfha)))
        
        #El algoritmo no funcion del todo bien ya que el alpha es constante de 90 grados, considero mi robot su sistema de referencia constante a 90 grados
        # y el algoritmo de descenso de gradiente solo funciona para el primer cuadrante al salirse del primer cuadrante del sistema de referencia tenemos problemas
        #La forma de corregir el error para que gire en la direccion correcta nuestro robotMovil es poniendo condiciones en funcion de fxRepulsion ya que este valor si nos tira la direccion correcta
        #La formula anterior nos deberia servir al momento de usar una IMU
        if FrepX<0:
            aux=wl
            wl=wr
            wr=aux
        print("wl: ", wl)
        print("wr: ", wr)
        vrT = wr*R
        
        vlT = wl*R
        
        vl=calcularVelocidadLineal(tiempoL)
        vr=calcularVelocidadLineal(tiempoR)
        #print("---------------------------------------VelocidadL:", vl)
        #print("---------------------------------------VelocidadR:", vr)
        
        wl=calcularVelocidadAngular(tiempoL)
        wr=calcularVelocidadAngular(tiempoR)
        
        e1Error = vlT - vl
        e2Error = vrT - vr

        print("----------------------------------------error=",e2Error-e1Error)
        
        m1Speed += (e1Error * KPL2) + (e1PrevError * KDL2) + (e1SumError * KIL2)
        m2Speed += (e2Error * KPR2)  + (e2PrevError * KDR2) + (e2SumError * KIR2)

        m1Speed = max(min(80, m1Speed), 20)
        m2Speed = max(min(80, m2Speed), 20)


        print("-----------------------------------------mL:",m1Speed)
        print("-----------------------------------------mR:",m2Speed)
        #print("--------------------------------------Desviando")

        motorPwm(m1Speed,m2Speed)
        e1PrevError = e1Error
        e2PrevError = e2Error

        e1SumError += e1Error
        e2SumError += e2Error
        estado="girando"
    else:
        if estado=='girando':
            e1PrevError=0
            e2PrevError=0
            e1SumError=0
            e2SumError=0
            estado='derecho'
        vl=calcularVelocidadLineal(tiempoL)
        vr=calcularVelocidadLineal(tiempoR)
        print("VelocidadL:", vl)
        print("VelocidadR:", vr)
        
        wl=calcularVelocidadAngular(tiempoL)
        wr=calcularVelocidadAngular(tiempoR)
        
        e1Error = TARGET - vl
        e2Error = TARGET - vr

        #print("error=",e2Error-e1Error)

        m1Speed += (e1Error * KPL) + (e1PrevError * KDL) + (e1SumError * KIL)
        m2Speed += (e2Error * KPR)  + (e2PrevError * KDR) + (e2SumError * KIR)

        m1Speed = max(min(80, m1Speed), 20)
        m2Speed = max(min(80, m2Speed), 20)


        #print("mL:",m1Speed)
        #print("mR:",m2Speed)
        print("Derecho")

        motorPwm(m1Speed,m2Speed)
        e1PrevError = e1Error
        e2PrevError = e2Error

        e1SumError += e1Error
        e2SumError += e2Error
        motorPwm(m1Speed,m2Speed)
