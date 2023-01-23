
import threading
import RPi.GPIO as GPIO
import time
from guizero import App, Box, Text, TextBox, PushButton, Slider
""" Input Output  configaration """
GPIO.setmode(GPIO.BCM)
GPIO.setup(26, GPIO.IN, GPIO.PUD_DOWN)
GPIO.setup(12, GPIO.OUT)                  # PWM pulse for MOSFET/IGBT 
GPIO.output(12, 0)
""" Global variabels  """
Set_RPM =500                              # SET RPM value
feedback=0.0                          
previous_time =0.0
previous_error=0.0
Integral=0.0           
D_cycal=10
Kp=0                                       # Proportional controller Gain (0 to 100)
Ki=0                                       # Integral controller Gain (0 to 100)
Kd=0                                       # Derivative controller Gain (0 to 100)
RunRPM=0
Loop_value=0
a=0
avr=0
i=0
GatePulse = GPIO.PWM(12, 100)
""" PID control function """
def PID_function():
    
    global previous_time
    global previous_error
    global Integral
    global D_cycal
    global Kp
    global Ki
    global Kd
    
    error = int(Set_RPM) -feedback                    # Differnce between expected RPM and run RPM
    
    if (previous_time== 0):
         previous_time =time.time()
         
    current_time = time.time()
    delta_time = current_time - previous_time
    delta_error = error - previous_error
    
    Pout = (Kp/10 * error)              
    
    Integral += (error * delta_time)
    
    
    if Integral>10:      
        Integral=10
        
    if Integral<-10:
        Integral=-10
    
    Iout=((Ki/10) * Integral)
    
    
    Derivative = (delta_error/delta_time)         #de/dt
    previous_time = current_time
    previous_error = error
    
    Dout=((Kd/1000 )* Derivative)
    
    output = Pout + Iout + Dout                  # PID controller output
    
    if ((output>D_cycal)&(D_cycal<90)):           
        D_cycal+=1
        
    if ((output<D_cycal)&(D_cycal>10)):           
        D_cycal-=1
        
    return ()
"""   RPM calculation function   """
 
def RPM_function():      
    global feedback
    tc=time.time()
    
    
    while (GPIO.input(26)==False):               
        v=0
        ts=time.time()
        time_count=ts-tc
        
        if (time_count>7):
            print("Feedback failed, Please make proper feedback connection")
            feedback=0
            return ()         
               
    
    while (GPIO.input(26)==True):                  
        i=0
        ts=time.time()
        time_count=ts-tc
        if (time_count>7):
            print("Feedback failed, Please make proper feedback connection")
            feedback=0
            return ()
    
    v = time.time()                              # Stores the first pulse time
    while (GPIO.input(26)==False):               
        s=0                                     
    while (GPIO.input(26)==True):                  
        h=0                                            
    h=time.time()                                # Stores the next pulse time
                                        
    w=(60/(h-v))                                 # MOTOR speed in RPM  
    feedback = w
    
    return ()
def main_function():
    global D_cycal
    
    if Loop_value==1:
        
        t1 = threading.Thread(target=RPM_function)
        t1.start()
        t1.join()
        t2 = threading.Thread(target=PID_function)
        t2.start()
        t2.join()
        
        GatePulse.ChangeDutyCycle(D_cycal)
        
    else:
        GatePulse.ChangeDutyCycle(0)
print("Welcome To The Element14 Community!")
"""     GUI Functions     """
app = App(title="Welcome To The Element14 Community", height=700, width=500)
def change_Kp_value(slider_value):
    global Kp
    Proportional.value = slider_value
    Kp=int(slider_value)
    
def change_Ki_value(slider_value):
    global Ki
    Intregral.value = slider_value
    Ki= int(slider_value)
    
def change_Kd_value(slider_value):
    global Kd
    Deravative.value = slider_value
    Kd=int(slider_value)
def change_SetRPM_value(slider_value):
    global Set_RPM
    SetRPM.value = slider_value
    Set_RPM=int(slider_value)
      
def update_rpm():
    global avr
    global i
    global a
    global feedback
    if(Loop_value==1):
        if i<6:
            a+=feedback
            i+=1
        else:
                        
            Run_RPM.value = int(a/6)
            a=0
            i=0
    else:
        Run_RPM.value = 0
    
def start_funcdtion():
    global Loop_value
    print("MOTOR controller is ON ")
    GatePulse.start(25)
    Loop_value=1
    Startbutton.toggle()
    Startbutton.toggle()
    Startbutton.repeat(1, main_function)
        
def Stop_function():
    
    global Loop_value
    Loop_value=0
    Startbutton.cancel(main_function)
    Run_RPM.value=00
    D_cycal=0
    Stopbutton.toggle()
    Stopbutton.toggle()
    GatePulse.ChangeDutyCycle(0)
    print("Motor controller is Off")
def close_window():
    Stop_function()
    app.hide()
message = Text(app, text="PID based DC motor controller", color="saddle brown", size= 20)
Startbutton = PushButton(app, command = start_funcdtion, text="Start")
Startbutton.text_color="dark green"
Startbutton.text_size=15
Spase = Text(app, text= "  ")
SetRPM = Text(app, text= "Set_RPM")
SetRPM.text_color="blue"
SetRPM = Text(app, text= Set_RPM)
text_value = Slider(app, command=change_SetRPM_value, start=500, end=2000)
Spase = Text(app, text= "  ")
Proportional = Text(app, text= "Kp")
Proportional.text_color="DarkOliveGreen4"
Proportional = Text(app, text= Kp)
text_value = Slider(app, command=change_Kp_value, start=0, end=100)
pase = Text(app, text= "  ")
Intregral = Text(app, text= "Ki")
Intregral.text_color="DarkOliveGreen4"
Intregral = Text(app, text= Ki)
text_value = Slider(app, command=change_Ki_value, start=0, end=100)
pase = Text(app, text= "  ")
Deravative = Text(app, text= "Kd")
Deravative.text_color="DarkOliveGreen4"
Deravative = Text(app, text= Kd)
text_value = Slider(app, command=change_Kd_value, start=0, end=100)
pase = Text(app, text= "  ")
Run_RPM = Text(app, text= "RunRPM")
Run_RPM.text_color="tomato"
Run_RPM = Text(app, text= RunRPM)
Run_RPM.repeat(1, update_rpm) 
pase = Text(app, text= "  ")
Stopbutton = PushButton(app, command = Stop_function, text="Stop")
Stopbutton.text_color="red"
Stopbutton.text_size=15
pase = Text(app, text= "  ")
close_button = PushButton(app, text="Close Project", command=close_window)
#close_button.text_coller="yellow"
close_button.text_size=20
app.display()