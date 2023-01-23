import RPi.GPIO as GPIO
import time
import os


ena_Derecha = 21 ######preguntar como esta cableado, de aca debe salir el pwm			
in1_Derecha = 3
in2_Derecha = 4

ena_Izquierda = 18 ######preguntar como esta cableado, de aca debe salir el pwm			
in1_Izquierda = 14
in2_Izquierda = 15

v_sonido = 34300 #cm/s velocidad del sonido

TRIGM = 23 #Variable que contiene el GPIO al cual conectamos la señal TRIG del sensor
ECHOM = 24 #Variable que contiene el GPIO al cual conectamos la señal ECHO del sensor

TRIGR = 25
ECHOR = 5

TRIGL = 22
ECHOL = 27

#################  VARIABLES PARA EL PID   ##################################

TIEMPO_MUESTREO=1 # ajustar en caso de necesidad

Vel_objetivo=15
KP=0.05 #valor inicial para prueba
KD=0.025
KI=0.0125
error_anterior_Derecha=0
error_anterior_Izquierda=0
sum_error_mas_error_ant_Derecha = 0
sum_error_mas_error_ant_Izquierda=0

pwm_P_Derecha=0
pwm_P_Izquierda=0
pwm_PD_Derecha=0
pwm_PD_Izquierda=0
pwm_PID_Derecha=0
pwm_PID_Izquierda=0
##################   VARIABLES USADAS PARA EL ENCODER  ##########################
Vel_derecha=0
Vel_izquierda=0
Pulsos_Rueda_Derecha=26
Pulsos_Rueda_Izquierda=17

GPIO.setmode(GPIO.BCM)#Modo de referenciar los pines
GPIO.setup(Pulsos_Rueda_Derecha, GPIO.IN)
GPIO.setup(Pulsos_Rueda_Izquierda, GPIO.IN)
#GPIO.add_event_detect(Pulsos_Rueda_Derecha, GPIO.RISING) # Entrada de los pulsos R. derecha
#GPIO.add_event_detect(Pulsos_Rueda_Izquierda, GPIO.RISING)#Entrada de los pulsos R.Izquierda
GPIO.setup(ena_Derecha,GPIO.OUT)
GPIO.setup(ena_Izquierda,GPIO.OUT)
GPIO.setup(in1_Derecha, GPIO.OUT)
GPIO.setup(in2_Derecha, GPIO.OUT)
GPIO.setup(in1_Izquierda, GPIO.OUT)
GPIO.setup(in2_Izquierda, GPIO.OUT)


GPIO.setup(TRIGM, GPIO.OUT) #Configuramos el pin TRIG como una salida 
GPIO.setup(ECHOM, GPIO.IN)  #Configuramos el pin ECHO como una salida 
GPIO.setup(TRIGR, GPIO.OUT)
GPIO.setup(ECHOR, GPIO.IN)
GPIO.setup(TRIGL, GPIO.OUT)
GPIO.setup(ECHOL, GPIO.IN)
GPIO.output(TRIGM, False)
GPIO.output(TRIGR, False)
GPIO.output(TRIGL, False)
print("Medicion de la distancia en curso")




print("Estabilizandose el sensor")
time.sleep(1)
print("Sensor estabilizado")
pwm_Derecha   = GPIO.PWM(ena_Derecha,100)
pwm_Izquierda = GPIO.PWM(ena_Izquierda,100)

def medirVelocidad(p,cuentaRueda):
    tiempo_inicial_derecha=time.time()       
    while True:
        if GPIO.input(p):                
            cuentaRueda += 1
            if (cuentaRueda==20):
                    tiempo_final_derecha=time.time()                       
                    break
    T20_derecha=tiempo_final_derecha - tiempo_inicial_derecha
    Vel_derecha =  (51836.278/T20_derecha)*(5/12)
    return Vel_derecha

def medirDistancia(ECHO, TRIG): 
        GPIO.output(TRIG, True) #Trig sensor medio alto
        time.sleep(0.00001) #Delay de 10 microsegundos
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
            return 31 
        
def imprimirDistancia(result, sensor):
    print("Distancia ",sensor,": ",result, " cm")
    
def condicionParada(num):
    if 30 > num:
        print("Now stop")
        #forward.stop() # stop PWM from GPIO output it is necessary
        #reverse.stop()
        #forward2.stop() # stop PWM from GPIO output it is necessary
        #reverse2.stop()
        #c=0
        while True:
            GPIO.output(ena_Derecha,GPIO.LOW)
            GPIO.output(ena_Izquierda,GPIO.LOW)
            #c=c+1
        #return True
while True:
    
    cont_Rueda_Derecha=0
    cont_Rueda_Izquierda=0
    
    #forward.start(0) 
    #reverse.start(0)
    #forward2.start(0) 
    #reverse2.start(0)
    
    GPIO.output(ena_Derecha,GPIO.HIGH)
    GPIO.output(ena_Izquierda,GPIO.HIGH)
    #forward.ChangeDutyCycle(0)
    #reverse.ChangeDutyCycle(100)
    #forward2.ChangeDutyCycle(0)
    #reverse2.ChangeDutyCycle(100)
    
    distancia1 = medirDistancia(ECHOM, TRIGM)
    imprimirDistancia(distancia1,"central")
    condicionParada(distancia1)
    
    distancia2 = medirDistancia(ECHOR, TRIGR)
    imprimirDistancia(distancia2,"derecha")
    condicionParada(distancia2)
    
    distancia3 = medirDistancia(ECHOL, TRIGL)
    imprimirDistancia(distancia3,"izquierda")
    condicionParada(distancia3)
    
    GPIO.output(ena_Derecha,GPIO.HIGH)
    GPIO.output(ena_Izquierda,GPIO.HIGH)
    
    pwm_Derecha.ChangeDutyCycle(100)
    pwm_Izquierda.ChangeDutyCycle(100)
    
    while True:
        
        distancia1 = medirDistancia(ECHOM, TRIGM)
        imprimirDistancia(distancia1,"central")
        condicionParada(distancia1)
        
        distancia2 = medirDistancia(ECHOR, TRIGR)
        imprimirDistancia(distancia2,"derecha")
        condicionParada(distancia2)
        
        distancia3 = medirDistancia(ECHOL, TRIGL)
        imprimirDistancia(distancia3,"izquierda")
        condicionParada(distancia3)
        
        Vel_derecha = medirVelocidad(26, cont_Rueda_Derecha)
        Vel_izquierda = medirVelocidad(17, cont_Rueda_Izquierda)
        
        error_derecha = Vel_objetivo - Vel_derecha
        error_izquierda = Vel_objetivo -Vel_izquierda
        pwm_P_Derecha += KP*error_derecha
        pwm_P_Izquierda += KP*error_derecha
        pwm_P_Derecha = max(min(1, pwm_P_Derecha), 0)
        pwm_P_Izquierda = max(min(1, pwm_P_Izquierda), 0)
        
        pwm_PD_Derecha += (error_derecha * KP) + (error_anterior_Derecha * KD)
        pwm_PD_Izquierda += (error_izquierda * KP) + (error_anterior_Izquierda * KD)
       
        error_anterior_Derecha=error_derecha
        error_anterior_Izquierda=error_izquierda

        pwm_PID_Derecha += (error_derecha * KP) + (error_anterior_Derecha * KD)+(sum_error_mas_error_ant_Derecha*KI)
        pwm_PID_Izquierda += (error_izquierda * KP) + (error_anterior_Izquierda * KD)+(sum_error_mas_error_ant_Izquierda*KI)
        pwm_PID_Derecha = max(min(1, pwm_PID_Derecha), 0)
        pwm_PID_Izquierda = max(min(1, pwm_PID_Izquierda), 0)
        time.sleep(TIEMPO_MUESTREO)
        sum_error_mas_error_ant_Derecha += error_derecha
        sum_error_mas_error_ant_Izquierda +=error_izquierda
        pwm_Derecha.ChangeDutyCycle(int(pwm_PID_Derecha))
        pwm_Izquierda.ChangeDutyCycle(int(pwm_PID_Izquierda))
