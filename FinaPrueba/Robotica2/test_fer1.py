import threading
from gpiozero import DigitalInputDevice, Robot
from time import sleep
import time
import math
import RPi.GPIO as GPIO
import asyncio

class PID_RASPY:
    def __init__(self, setpoint,Sample_time, KD,KP,KI):
        self.Setpoint= setpoint
        self.KD= KD
        self.KP= KP
        self.KI= KI
        self.Sample_time= Sample_time
        self.last_Time=0
        #self.I_term=0
        #self.Out_max=100
        #self.Out_min=20
        #self.last_input=0
        #self.output=0
    def calcular_esfuerzo(self):
        tiempo_actual=time.time()
        variacion_tiempo= tiempo_actual - self.last_Time
        #return variacion_tiempo
        if(variacion_tiempo >= self.Sample_time):
            print('tiempo de muestreo cumplido')
            #error= self.Setpoint - entrada
            #self.I_term += self.KI* error
            #if self.I_term > self.Out_max :
                #self.I_term = self.Out_max
            
            #if self.I_term < self.Out_min:
                #self.I_term = self.Out_min
        
            #dInput = entrada - self.last_input
           
            #self.output= self.KP*error + self.I_term - self.KD*dInput
            
            #if self.output > self.Out_max :
                #self.output=self.Out_max
                
            #if self.output < self.Out_min :
                #self.output=self.Out_min
            
            #self.last_input = entrada
            #self.last_Time  = tiempo_actual
            #return output
        
        
            
            


objeto_prueba=PID_RASPY(20,10,5,10,5)
print(objeto_prueba.calcular_esfuerzo())


        
        