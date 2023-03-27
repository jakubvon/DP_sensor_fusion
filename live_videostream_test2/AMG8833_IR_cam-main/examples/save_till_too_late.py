#!/usr/bin/python
import RPi.GPIO as GPIO
import time
from array import *

people = 0
print ( "Number of people:",people)
timeout = 0
order = "3"

pin_trigger = array('i',[16,13])
pin_echo = array('i',[18,15])

PIN_TRIGGER = 16
PIN_ECHO = 18
PIN_TRIGGER_2 = 13
PIN_ECHO_2 = 15


GPIO.setmode(GPIO.BOARD)
GPIO.setup(PIN_TRIGGER, GPIO.OUT)
GPIO.setup(PIN_ECHO, GPIO.IN)
GPIO.setup(PIN_TRIGGER_2, GPIO.OUT)
GPIO.setup(PIN_ECHO_2, GPIO.IN)

def measureDistance(sensor_number):

      GPIO.output(pin_trigger[sensor_number], GPIO.LOW)
      time.sleep(0.000002) #print ( "Waiting for sensor to settle")
      GPIO.output(pin_trigger[sensor_number], GPIO.HIGH) #print ( "Calculating distance")
      time.sleep(0.00001)
      GPIO.output(pin_trigger[sensor_number], GPIO.LOW)

      while GPIO.input(pin_echo[sensor_number])==0:
            pulse_start_time = time.time()
      while GPIO.input(pin_echo[sensor_number])==1:
            pulse_end_time = time.time()

      pulse_duration = pulse_end_time - pulse_start_time
      distance = round(pulse_duration * 17150, 2)

      return distance








time.sleep(0.5)
GPIO.output(PIN_TRIGGER, GPIO.LOW)
time.sleep(0.000002) #print ( "Waiting for sensor to settle")
GPIO.output(PIN_TRIGGER, GPIO.HIGH) #print ( "Calculating distance")
time.sleep(0.00001)
GPIO.output(PIN_TRIGGER, GPIO.LOW)

while GPIO.input(PIN_ECHO)==0:
        pulse_start_time = time.time()
while GPIO.input(PIN_ECHO)==1:
        pulse_end_time = time.time()

pulse_duration = pulse_end_time - pulse_start_time
distance1_initial = round(pulse_duration * 17150, 2)
#print ( "Distance of first sensor:",distance,"cm")

GPIO.output(PIN_TRIGGER_2, GPIO.LOW) 
time.sleep(0.000002) #print ( "Waiting for sensor to settle")
GPIO.output(PIN_TRIGGER_2, GPIO.HIGH) #print ( "Calculating distance")
time.sleep(0.00001)
GPIO.output(PIN_TRIGGER_2, GPIO.LOW)

while GPIO.input(PIN_ECHO_2)==0:
        pulse2_start_time = time.time()
while GPIO.input(PIN_ECHO_2)==1:
        pulse2_end_time = time.time()

pulse2_duration = pulse2_end_time - pulse2_start_time
distance2_initial = round(pulse2_duration * 17150, 2)
#print ( "Distance of second sensor:",distance2,"cm")

if distance1_initial>250:
    distance1_initial=250

if distance2_initial>250:
    distance2_initial=250



while(1):
      GPIO.setmode(GPIO.BOARD)
      GPIO.setup(PIN_TRIGGER, GPIO.OUT)
      GPIO.setup(PIN_ECHO, GPIO.IN)
      GPIO.setup(PIN_TRIGGER_2, GPIO.OUT)
      GPIO.setup(PIN_ECHO_2, GPIO.IN)

      GPIO.output(PIN_TRIGGER, GPIO.LOW)
      time.sleep(0.000002) #print ( "Waiting for sensor to settle")
      GPIO.output(PIN_TRIGGER, GPIO.HIGH) #print ( "Calculating distance")
      time.sleep(0.00001)
      GPIO.output(PIN_TRIGGER, GPIO.LOW)

      while GPIO.input(PIN_ECHO)==0:
            pulse_start_time = time.time()
      while GPIO.input(PIN_ECHO)==1:
            pulse_end_time = time.time()

      pulse_duration = pulse_end_time - pulse_start_time
      distance1 = round(pulse_duration * 17150, 2)
      #print ( "Distance of first sensor:",distance,"cm")

      GPIO.output(PIN_TRIGGER_2, GPIO.LOW) 
      time.sleep(0.000002) #print ( "Waiting for sensor to settle")
      GPIO.output(PIN_TRIGGER_2, GPIO.HIGH) #print ( "Calculating distance")
      time.sleep(0.00001)
      GPIO.output(PIN_TRIGGER_2, GPIO.LOW)

      while GPIO.input(PIN_ECHO_2)==0:
            pulse2_start_time = time.time()
      while GPIO.input(PIN_ECHO_2)==1:
            pulse2_end_time = time.time()

      pulse2_duration = pulse2_end_time - pulse2_start_time
      distance2 = round(pulse2_duration * 17150, 2)
      #print ( "Distance of second sensor:",distance2,"cm")

      if distance1>250:
        distance1=250

      if distance2>250:
        distance2=250

      if distance1<(distance1_initial-50) and order[0] != "1":
            if order == "3":
                  order = ""
            order += "1"

      elif distance2<(distance2_initial-50) and order[0] != "2":
            if order == "3":
                  order = ""
            order += "2"

      if order == "12":
            if timeout<10:
                  order="3"
            else:
                  people = people + 1
                  order = "3"
                  print ( "Number of people:",people)
                  time.sleep(2)

      elif order == "21" and people > 0:
            if timeout<10:
                  order = "3"
            else:
                  people = people - 1
                  order = "3"
                  print ( "Number of people:",people)
                  time.sleep(2)

      elif order == "21" and people == 0:
            order = "3"

      if len(order)>2 or order == "11" or order == "22" or timeout>1000:
            order = "3"
            timeout = 0        

      if len(order)==1 and order!="3":
            timeout = timeout+1
      else:
            timeout = 0

      print(distance1)
      GPIO.cleanup()