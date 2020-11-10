
from signal import pause
import RPi.GPIO as GPIO
import time
import paho.mqtt.client as mqtt
import smbus
import math
from time import sleep
from time import sleep as delay
import time
import argparse
from rpi_ws281x import *

clp = 20
servoPin          = 12
SERVO_MAX_DUTY    = 12   
SERVO_MIN_DUTY    = 3

power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c
buzzer=21
TRIG = 22
ECHO = 27
GPIO.setwarnings(False)
GPIO.setmode (GPIO.BCM)
GPIO.setup(clp,GPIO.IN)
GPIO.setup (14, GPIO.OUT)
GPIO.setup (15, GPIO.OUT)
GPIO.setup (23, GPIO.OUT)
GPIO.setup (24, GPIO.OUT)
GPIO.setup (5, GPIO.OUT)
GPIO.setup (6, GPIO.OUT)
GPIO.setup (13, GPIO.OUT)
GPIO.setup(TRIG,GPIO.OUT)
GPIO.setup(ECHO,GPIO.IN)
GPIO.output(TRIG,False)
LED_COUNT      = 10      # Number of LED pixels.
LED_PIN        = 18      # GPIO pin connected to the pixels (18 uses PWM!).
#LED_PIN        = 10      # GPIO pin connected to the pixels (10 uses SPI /dev/spidev0.0).
LED_FREQ_HZ    = 800000  # LED signal frequency in hertz (usually 800khz)
LED_DMA        = 10      # DMA channel to use for generating signal (try 10)
LED_BRIGHTNESS = 255     # Set to 0 for darkest and 255 for brightest
LED_INVERT     = False   # True to invert the signal (when using NPN transistor level shift)
LED_CHANNEL    = 0 
GPIO.setup(buzzer,GPIO.OUT)
GPIO.setup(servoPin, GPIO.OUT)  
servo = GPIO.PWM(servoPin, 50)  
servo.start(0)





    
def aaa():
        GPIO.output(TRIG, True)
        time.sleep(0.0001)
        GPIO.output(TRIG, False)
    
        
        while GPIO.input(ECHO)==0:
            start= time.time()
        while GPIO.input(ECHO)==1:
            stop= time.time()

        check_time = stop - start
        distance = check_time * 34300 / 2
        print("distance:%.1f cm" % distance)
        buzzer=21
        
        
        
        
        if distance < 15:
            print ("guaa")
            
            GPIO.output(buzzer,GPIO.HIGH)
           
           
            
           
            ddd()
        
            
        else:
            GPIO.output(buzzer,GPIO.LOW)
            eee()
        
           


def colorWipe(strip, color, wait_ms=50):
    """Wipe color across display a pixel at a time."""
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, color)
        strip.show()
        time.sleep(wait_ms/1000.0)

def theaterChase(strip, color, wait_ms=50, iterations=10):
    """Movie theater light style chaser animation."""
    for j in range(iterations):
        for q in range(3):
            for i in range(0, strip.numPixels(), 3):
                strip.setPixelColor(i+q, color)
            strip.show()
            time.sleep(wait_ms/1000.0)
            for i in range(0, strip.numPixels(), 3):
                strip.setPixelColor(i+q, 0)

def wheel(pos):
    """Generate rainbow colors across 0-255 positions."""
    if pos < 85:
        return Color(pos * 3, 255 - pos * 3, 0)
    elif pos < 170:
        pos -= 85
        return Color(255 - pos * 3, 0, pos * 3)
    else:
        pos -= 170
        return Color(0, pos * 3, 255 - pos * 3)

def rainbow(strip, wait_ms=20, iterations=1):
    """Draw rainbow that fades across all pixels at once."""
    for j in range(256*iterations):
        for i in range(strip.numPixels()):
            strip.setPixelColor(i, wheel((i+j) & 255))
        strip.show()
        time.sleep(wait_ms/1000.0)

def rainbowCycle(strip, wait_ms=20, iterations=5):
    """Draw rainbow that uniformly distributes itself across all pixels."""
    for j in range(256*iterations):
        for i in range(strip.numPixels()):
            strip.setPixelColor(i, wheel((int(i * 256 / strip.numPixels()) + j) & 255))
        strip.show()
        time.sleep(wait_ms/1000.0)

def theaterChaseRainbow(strip, wait_ms=50):
    """Rainbow movie theater light style chaser animation."""
    for j in range(256):
        for q in range(3):
            for i in range(0, strip.numPixels(), 3):
                strip.setPixelColor(i+q, wheel((i+j) % 255))
            strip.show()
            time.sleep(wait_ms/1000.0)
            for i in range(0, strip.numPixels(), 3):
                strip.setPixelColor(i+q, 0)
def read_byte(adr):
    return bus.read_byte_data(address, adr)

def read_word(adr):
    high = bus.read_byte_data(address, adr)
    low = bus.read_byte_data(address, adr+1)
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def dist(a,b):
    return math.sqrt((a*a)+(b*b))

def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)

def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)

    


bus = smbus.SMBus(1) 
address = 0x68     
bus.write_byte_data(address, power_mgmt_1, 0)

def ccc():
    pin_read = GPIO.input(clp)
    print(pin_read)
    if pin_read == 1:
        print("collision")

        

def bbb():
    print ("gyro data")
    
    gyro_xout = read_word_2c(0x43)
    gyro_yout = read_word_2c(0x45)
    gyro_zout = read_word_2c(0x47)
    
     

    print ("gyro_xout: ", gyro_xout, " scaled: ", (gyro_xout / 131))
    print ("gyro_yout: ", gyro_yout, " scaled: ", (gyro_yout / 131))
    print ("gyro_zout: ", gyro_zout, " scaled: ", (gyro_zout / 131))

    
    print ("accelerometer data")
    

    accel_xout = read_word_2c(0x3b)
    accel_yout = read_word_2c(0x3d)
    accel_zout = read_word_2c(0x3f)

    accel_xout_scaled = accel_xout / 16384.0
    accel_yout_scaled = accel_yout / 16384.0
    accel_zout_scaled = accel_zout / 16384.0

    print ("accel_xout: ", accel_xout, " scaled: ", accel_xout_scaled)
    print ("accel_yout: ", accel_yout, " scaled: ", accel_yout_scaled)

    print ("accel_zout: ", accel_zout, " scaled: ", accel_zout_scaled)
    print ("x rotation: " , get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled))
    print ("y rotation: " , get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled))
    time.sleep(1)
    
    yrotation = get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled) 
    if yrotation > 4 or yrotation < 2:
            
            print("buaaaa")
            GPIO.output(buzzer,GPIO.HIGH)
            GPIO.output(13,False)
            GPIO.output(5,True)
    else:
        
        print("cuaaa")
        GPIO.output(buzzer,GPIO.LOW)
        GPIO.output(5,False)
        GPIO.output(13,True)
    
def setServoPos(degree):

  if degree > 180:
    degree = 180

  
  duty = SERVO_MIN_DUTY+(degree*(SERVO_MAX_DUTY-SERVO_MIN_DUTY)/180.0)
  
  print("Degree: {} to {}(Duty)".format(degree, duty))

 
  servo.ChangeDutyCycle(duty)
  
  


parser = argparse.ArgumentParser()
parser.add_argument('-c', '--clear', action='store_true', help='clear the display on exit')
args = parser.parse_args()

    # Create NeoPixel object with appropriate configuration.
strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
    # Intialize the library (must be called once before other functions).
strip.begin()

def ddd():
            colorWipe(strip, Color(255, 0, 0))  # Red wipe
            colorWipe(strip, Color(0, 255, 0))  # Blue wipe
            colorWipe(strip, Color(0, 0, 255))
            return
        

def eee():
            colorWipe(strip, Color(0, 0, 0))  # Red wipe
            colorWipe(strip, Color(0, 0, 0))  # Blue wipe
            colorWipe(strip, Color(0, 0, 0))
            return
    

while True:
    try:
        aaa()
        bbb()
        
        
        
        
    except:
        print("aa")
        
    

  
      
        
   
        
        
    
        
   


    




