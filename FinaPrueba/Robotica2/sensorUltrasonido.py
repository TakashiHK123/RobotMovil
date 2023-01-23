import RPi.GPIO as GPIO
import time

TRIG = 23 #Variable que contiene el GPIO al cual conectamos la señal TRIG del sensor
ECHO = 24 #Variable que contiene el GPIO al cual conectamos la señal ECHO del sensor

TRIGR = 25
ECHOR = 5

TRIGL = 22
ECHOL = 27

GPIO.setmode(GPIO.BCM)     #Establecemos el modo según el cual nos refiriremos a los GPIO de nuestra RPi            
GPIO.setup(TRIG, GPIO.OUT) #Configuramos el pin TRIG como una salida 
GPIO.setup(ECHO, GPIO.IN)  #Configuramos el pin ECHO como una salida 
GPIO.setup(TRIGR, GPIO.OUT)
GPIO.setup(ECHOR, GPIO.IN)
GPIO.setup(TRIGL, GPIO.OUT)
GPIO.setup(ECHOL, GPIO.IN)

#Contenemos el código principal en un aestructura try para limpiar los GPIO al terminar o presentarse un error
try:
    #Implementamos un loop infinito
    while True:
        
        # Ponemos en bajo el pin TRIG y después esperamos 0.5 seg para que el transductor se estabilice
        GPIO.output(TRIG, GPIO.LOW)
        GPIO.output(TRIGR, GPIO.LOW)
        GPIO.output(TRIGL, GPIO.LOW)
        time.sleep(0.5)

        #Ponemos en alto el pin TRIG esperamos 10 uS antes de ponerlo en bajo
        GPIO.output(TRIG, GPIO.HIGH)
        GPIO.output(TRIGR, GPIO.HIGH)
        GPIO.output(TRIGL, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(TRIG, GPIO.LOW)
        GPIO.output(TRIGR, GPIO.LOW)
        GPIO.output(TRIGL, GPIO.LOW)
        


        # En este momento el sensor envía 8 pulsos ultrasónicos de 40kHz y coloca su pin ECHO en alto
        # Debemos detectar dicho evento para iniciar la medición del tiempo
        
        while True:
            pulso_inicio = time.time()
            if GPIO.input(ECHO) == GPIO.HIGH:
                break
            
        # El pin ECHO se mantendrá en HIGH hasta recibir el eco rebotado por el obstáculo. 
        # En ese momento el sensor pondrá el pin ECHO en bajo.
	# Prodedemos a detectar dicho evento para terminar la medición del tiempo
        
        while True:
            pulso_fin = time.time()
            if GPIO.input(ECHO) == GPIO.LOW:
                break
            
            
        #while True:
            #pulso_r_inicio = time.time()
            #if GPIO.input(ECHOR) == GPIO.HIGH:
                #break
            
        #while True:
            pulso_r_fin = time.time()
            #if GPIO.input(ECHOR) == GPIO.LOW:
                #break
            
        #while True:
            #pulso_l_inicio= time.time()
            #if GPIO.input(ECHOL) == GPIO.HIGH:
                #break
            
        #while True:
            #pulso_l_fin = time.time()
            #if GPIO.input(ECHOL) == GPIO.LOW:
                #break
        # Tiempo medido en segundos
        duracion = pulso_fin - pulso_inicio
        #duracionR = pulso_r_fin - pulso_r_inicio
        #duracionL = pulso_l_fin - pulso_l_inicio

        #Obtenemos la distancia considerando que la señal recorre dos veces la distancia a medir y que la velocidad del sonido es 343m/s
        distancia = (34300 * duracion) / 2
        #distanciaR = (34300 * duracionR) / 2
        #distanciaL = (34300 * duracionL) / 2
        # Imprimimos resultado
        print( "Distancia: %.2f cm" % distancia)
        #print("Distancia: %.2f cm" % distanciaR)
        #print("Distancia: %.2f cm" % distanciaL)

finally:
    # Reiniciamos todos los canales de GPIO.
    GPIO.cleanup()
    