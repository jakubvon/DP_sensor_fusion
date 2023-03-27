#!/usr/bin/python

#import libraries
import RPi.GPIO as GPIO
import time
from array import *

#define important variables
people = 0                                #number of people in the room counted
print ( "Number of people:",people)
timeout = 0                               #timeout - used so it doesnt count people when only standing near door
order = "3"                               #3 - default, 12 - someone stepped in, 21-someone stepped out, other - not ok ->reset

pin_trigger = array('i',[16,13])          #trigger pins on RPi of both sensors
pin_echo = array('i',[18,15])             #echo pins on RPi of both sensors


#setup pins and board mode

GPIO.setmode(GPIO.BOARD)
GPIO.setup(pin_trigger[0], GPIO.OUT)
GPIO.setup(pin_echo[0], GPIO.IN)
GPIO.setup(pin_trigger[1], GPIO.OUT)
GPIO.setup(pin_echo[1], GPIO.IN)

def measureDistance(sensor_number):

      global pulse_start_time
      global pulse_end_time

      #trigger -> wait for echo

      GPIO.output(pin_trigger[sensor_number], GPIO.LOW)
      time.sleep(0.000002)
      GPIO.output(pin_trigger[sensor_number], GPIO.HIGH)
      time.sleep(0.00001)
      GPIO.output(pin_trigger[sensor_number], GPIO.LOW)

      while GPIO.input(pin_echo[sensor_number])==0:
            pulse_start_time = time.time()
      while GPIO.input(pin_echo[sensor_number])==1:
            pulse_end_time = time.time()

      #calculate distance from echo time

      pulse_duration = pulse_end_time - pulse_start_time
      distance = round(pulse_duration * 17150, 2)

      return distance

#initial distance values => later comapre them with actual measurements for people counting

distance1_initial = measureDistance(0)
distance2_initial = measureDistance(1)

#HC-SR04 sensors only measure well till 2-2.5m -> if more, we round it to max distance

if distance1_initial>250:
    distance1_initial=250

if distance2_initial>250:
    distance2_initial=250

#main loop

while(1):

      # measure distances by both sensors and round down if necessary

      distance1 = measureDistance(0)
      distance2 = measureDistance(1)

      if distance1>250:
        distance1=250

      if distance2>250:
        distance2=250


      if distance1<(distance1_initial-30) and order[0] != "1": #if change in distance by sensor1 and it hasnt been already reported in previous rounds => write 1 in the sequence first place
            if order == "3":
                  order = ""
            order += "1"

      elif distance2<(distance2_initial-30) and order[0] != "2": #if change in distance by sensor1 and it hasnt been already reported in previous rounds => write 2 in the sequence first place
            if order == "3":
                  order = ""
            order += "2"

      if order == "12":                                     #first activated sensor1,then sensor2 -> someone came in the room
            if timeout<10:                                  # makes sure that noone is just standing in front of the sensors
                  order="3"                                 # change sequence to default
            else:
                  people = people + 1                       # counts +1 person
                  order = "3"                               # change sequence to default
                  print ( "Number of people:",people)       
                  time.sleep(1.5)                             # waits till he is gone

      elif order == "21" and people > 0:                    #first activated sensor2,then sensor1 -> someone left the room (+cant be less than 0)
            if timeout<10:
                  order = "3"
            else:
                  people = people - 1                       # counts -1 person
                  order = "3"
                  print ( "Number of people:",people)
                  time.sleep(1.5)

      elif order == "21" and people == 0:                   #cant be less than 0 -> reset sequence, still 0 people
            order = "3"

      if len(order)>2 or order == "11" or order == "22" or timeout>1000:            #fault sequences or timeout => reset
            order = "3"
            timeout = 0        

      if len(order)==1 and order!="3":                                        #if one sensor detected, start timeout count
            timeout = timeout+1
      else:
            timeout = 0                                                       #if sequence alreadz reset => reset timeout

      #GPIO.cleanup()                                                          # reset GPIOs