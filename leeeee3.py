from bluedot import BlueDot
from signal import pause
import RPi.GPIO as GPIO
import time
from time import sleep




GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
control_pins = [7,8,16,25]

GPIO.setup (7, GPIO.OUT)
GPIO.setup (8, GPIO.OUT)
GPIO.setup (25, GPIO.OUT)
GPIO.setup (16, GPIO.OUT)

GPIO.setup (14, GPIO.OUT)
GPIO.setup (15, GPIO.OUT)
GPIO.setup (23, GPIO.OUT)
GPIO.setup (24, GPIO.OUT)


for pin in control_pins:

  GPIO.setup(pin,GPIO.OUT)

  GPIO.output(pin,False)






 



StepCount = 4



halfstep_seq = [
    [1,0,0,0],
    [1,1,0,0],
    [0,1,0,0],
    [0,1,1,0],
    [0,0,1,0],
    [0,0,1,1],
    [0,0,0,1],
    [1,0,0,1],
]


halfstep_seq2 = [
    [1,0,0,1],
    [0,0,0,1],
    [0,0,1,1],
    [0,0,1,0],
    [0,0,1,0],
    [0,1,0,0],
    [1,1,0,0],
    [1,0,0,0],
]















 







def eee():
    for i in range(512):
        for halfstep in range(8):
            for pin in range(4):
                GPIO.output(control_pins[pin],halfstep_seq[halfstep][pin])
            time.sleep(0.001)
        

def fff():
    for i in range(512):
        for halfstep in range(8):
            for pin in range(4):
                GPIO.output(control_pins[pin],halfstep_seq2[halfstep][pin])
            time.sleep(0.001)


def uuu():
    StepCounter = 0 
    while True:
        for pin in range(0, 4):
            xpin = StepPins[pin]

            if Seq3[StepCounter][pin]!=0:
                GPIO.output(xpin, True)

            else:
                GPIO.output(xpin, False)

        StepCounter += 1

        if (StepCounter==StepCount):
            break
        

        if (StepCounter<5):
            break
            

        time.sleep(0.01)

            
     
    


    
                
    
    
    

        

                         

            
    
        
    
    



    

    
    
    
    
    
    
    
    









def forward():
    GPIO.output (23, 0)
    GPIO.output (24, 1) 
    GPIO.output (14, 1)
    GPIO.output (15, 0)

    
                 
def back():
    GPIO.output (23, 1)
    GPIO.output (24, 0)
    GPIO.output (14, 0)
    GPIO.output (15, 1)
    
def left():
    GPIO.output (23, 0)
    GPIO.output (24, 1)
    GPIO.output (14, 0)
    GPIO.output (15, 0)
def lll():
    GPIO.output (23, 0)
    GPIO.output (24, 0)
    GPIO.output (14, 1)
    GPIO.output (15, 0)
def stop():
    GPIO.output (23, 0)
    GPIO.output (24, 0)
    GPIO.output (14, 0)
    GPIO.output (15, 0)
def dpad(pos):
    
    if pos.top:
     
        forward()
    elif pos.bottom:
    
        back()
    elif pos.left:
    
        left()
    elif pos.right:
       #    print ("kanan")
        lll()
        
    elif pos.middle:
    # print ("stop")
        stop()
        
      
        
        
        
    elif pos.lee:
        print("guaa")
        eee()
        
    elif pos.lee2:
        print("suaa")
        fff()
        

        

bd = BlueDot()
bd.when_pressed = dpad

pause()
GPIO.cleanup()

