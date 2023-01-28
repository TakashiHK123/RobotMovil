#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from std_msgs.msg import Float32
import concurrent.futures
import time
import math
import RPi.GPIO as GPIO

global fx,fy,frepX

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
#Condicion de minimo local
minimoFocal=20


#Valores para controlar movimiento
R=3.3 #Radio de la rueda 3.3cm
b=9 #Distancia del punto de contacto de la rueda al punto medio del eje

GPIO.setmode(GPIO.BCM)
GPIO.setup(ENCODER_LA,GPIO.IN)  #Definimos los pin de encoders como entrada
GPIO.setup(ENCODER_LB,GPIO.IN)
GPIO.setup(ENCODER_RA,GPIO.IN)
GPIO.setup(ENCODER_RB,GPIO.IN)

GPIO.add_event_detect(ENCODER_LA, GPIO.FALLING)#Pin de encoder definidos para detectar eventos 
GPIO.add_event_detect(ENCODER_LB, GPIO.FALLING)
GPIO.add_event_detect(ENCODER_RA, GPIO.FALLING)
GPIO.add_event_detect(ENCODER_RB, GPIO.FALLING)


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



#Controlador del motor por PWM 
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
    
    
#Condicion de parada se puede ajustar el valor de la distancia para detener para ultimo recurso.
def condicionParada(num):
    if 8 > num:
        print("Now stop")
        motorPwm(0,0)
        time.sleep(3)
        motorPwm(0,20)
        time.sleep(3)
        return True
        
def contador(count):
    count=count+1
    return count


def countEncoder(encoder,timeAnterior):  #Calcula el tiempo cada 30 pulsos del encoder, asi para poder calcular la velocidad del robotMovil 
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
    
def velocidadAngular(wl,wr): #Calculamos la velocidadAngular de giro del robot 
    w=(R*(wr-wl))*2*b
    return w


    

def velocidadCentroRobot(wl,wr): #Velocidad lineal centro robot, entre el punto medio del eje del robot 
    v=(R*(wl+wr))/2
    return v

#Velocidad de un punto descentrado a una distancia e del eje del robot 
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

#Velocidad del centro del eje del robot
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


def dataFx(data):
    fx=data
def dataFy(data):
    fy=data
def dataFrepX(data):
    frepX=data 
    
#Inicializamos unos valores para comenzar con el main 
#-------------------MAIN----------------------------
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
fx=None
fy=None
frepX=None
while True:

    #Metodo para la ejecucion de los encoders de forma paralela 
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
            print('se a generado una exception')

    #Una vez que termina la ejecución paralela continua la ejecucion de forma concurrente. 
    
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

    #motorPwm(m1Speed,m2Speed)
    e1PrevError = e1Error
    e2PrevError = e2Error

    e1SumError += e1Error
    e2SumError += e2Error
    
    
    rospy.init_node('controlador', anonymous=True)

    rospy.Subscriber('fx', Float32, dataFx)
    rospy.Subscriber('fy',Float32, dataFy)
    rospy.Subscriber('frepX',Float32, dataFrepX)

    #Calculamos las gradientes de repulsion y seran 3 por los tres sensores que son 3 obstaculos detectables por el robot
   
    
        #Aplicamos el descenso por gradiente para sacarl las velocidades angulares para girar 
    if distanciaM<=minimoFocal and distanciaL<=minimoFocal and distanciaR<=minimoFocal:
        e1Error = 0 - vl
        e2Error = 0 - vr
        
        m1Speed += (e1Error * KPL) + (e1PrevError * KDL) + (e1SumError * KIL)
        m2Speed += (e2Error * KPR)  + (e2PrevError * KDR) + (e2SumError * KIR)

        m1Speed = max(min(30, m1Speed), 0)
        m2Speed = max(min(30, m2Speed), 0)


        print("Frenando")

        motorPwm(m1Speed,m2Speed)
        print("---------------------m1: ", m1Speed, "--------------m2: ",)
        e1PrevError = e1Error
        e2PrevError = e2Error

        e1SumError += e1Error
        e2SumError += e2Error
        condicionParada(distanciaM)
            
            
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
        if frepX<0:
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