#!/usr/bin/env python3

import RPi.GPIO as GPIO     
from time import sleep 

GPIO.setmode(GPIO.BCM)
servo_pin = 18                  

GPIO.setup(servo_pin, GPIO.OUT)  
servo = GPIO.PWM(servo_pin, 50)  
servo.start(0) 

servo_min_duty = 3               
servo_max_duty = 12             

def set_servo_degree(degree):    
    if degree > 180:
        degree = 180
    elif degree < 0:
        degree = 0

    duty = servo_min_duty+(degree*(servo_max_duty-servo_min_duty)/180.0)
  
    GPIO.setup(servo_pin, GPIO.OUT)
    servo.ChangeDutyCycle(duty)
    sleep(0.3)                        
    GPIO.setup(servo_pin, GPIO.IN)  
try:                                    
    while True:                         
        set_servo_degree(0)             
        sleep(1)                        
        set_servo_degree(90)
        sleep(1)
        set_servo_degree(180)
        sleep(1)
        set_servo_degree(90)
        sleep(1)

finally:                               
    GPIO.cleanup()                     
