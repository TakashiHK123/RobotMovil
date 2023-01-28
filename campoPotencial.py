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
import time
import math

from std_msgs.msg import Float32
global distanciaL, distanciaM,distanciaR
distanciaM=100
distanciaL=100
distanciaR=100
#Valores del punto descentrado
e=13 #13cm del centro del eje de giro
alfha=math.pi/2 #90grados en radianes consideramos siempre nuestro eje en el centro del robot 
#Valores de repulsion
er=0.0028 #Constante ajustable
Qmin=1
Qinf=45
#Condicion de minimo local
minimoFocal=20
#Inicializador de variables para nuestros sensor de distancia 
inicioPulso=0
finPulso=0
ed = 0.003 #constante ajustable
xd=0
yd=200


def sensorL(data):
    print('Sensor Izquierda:' ,data)
    distanciaL=data

def sensorM(data):
    print('Sensor Medio:', data)
    distanciaM=data

def sensorR(data):
    print('Sensor Derecho:', data)
    distanciaR=data 

#Calculo de la gradiente de repulsion 
def gradienteDeRepulsion(l,distanciaObstaculo):  #La l es la distancia con respecto al eje x o y
    if distanciaObstaculo>=Qmin and distanciaObstaculo<=Qinf:  #Condicion de proximidad 
        gradienteRepulsion=(er*l)/(distanciaObstaculo**3)
    else:
        gradienteRepulsion=0
    return gradienteRepulsion

def fuerzaDeRepulsion(gradienteRepulsion): #Fuerza de repulsion, que equivale el signo negativo de la gradiente 
    Frepul=(-gradienteRepulsion)
    return Frepul

def fuerzaDeAtraccion(dd): #nuestra fuera de atraccion es en linea recta a una distancia fija en y 
    Fatrac=ed*dd
    return Fatrac
#Fuerza sumatoria de la fuerza de atraccion con la fuerza de repulsion
def fuerzaX(FatracX,FrepX,FrepY,alpha):
    fx=FatracX + FrepX*math.cos(alpha) - FrepY*math.sin(alpha)
    return fx

def fuerzaY(FatracY,FerpX,FrepY,alpha):
    fy=FatracY + FerpX*math.sin(alpha) + FrepY*math.cos(alpha)
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

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('campoPotencial', anonymous=True)

    rospy.Subscriber('sensorL', Float32, sensorL)
    rospy.Subscriber('sensorR', Float32, sensorR)
    rospy.Subscriber('sensorM', Float32, sensorM)

    fxP = rospy.Publisher('fx', Float32, queue_size=10)
    fyP = rospy.Publisher('fy', Float32, queue_size=10)
    frepXP = rospy.Publisher('frepX', Float32, queue_size=10)
    rate = rospy.Rate(10) # 10hz

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
        fxP.publish(fx)
        fyP.publish(fy)
        frepXP.publish(FrepX)
        rate.sleep()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
