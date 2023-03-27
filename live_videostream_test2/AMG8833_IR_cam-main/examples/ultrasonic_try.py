#!/usr/bin/python
import RPi.GPIO as GPIO
import time

people = 0

while(1):
      GPIO.setmode(GPIO.BOARD)

      PIN_TRIGGER = 16
      PIN_ECHO = 18
      PIN_TRIGGER_2 = 13
      PIN_ECHO_2 = 15

      GPIO.setup(PIN_TRIGGER, GPIO.OUT)
      GPIO.setup(PIN_ECHO, GPIO.IN)

      GPIO.output(PIN_TRIGGER, GPIO.LOW)

      #print ( "Waiting for sensor to settle")

      time.sleep(0.05)

      #print ( "Calculating distance")

      GPIO.output(PIN_TRIGGER, GPIO.HIGH)

      time.sleep(0.00001)

      GPIO.output(PIN_TRIGGER, GPIO.LOW)

      while GPIO.input(PIN_ECHO)==0:
            pulse_start_time = time.time()
      while GPIO.input(PIN_ECHO)==1:
            pulse_end_time = time.time()

      pulse_duration = pulse_end_time - pulse_start_time
      distance = round(pulse_duration * 17150, 2)
      #print ( "Distance of first sensor:",distance,"cm")

      GPIO.setup(PIN_TRIGGER_2, GPIO.OUT)
      GPIO.setup(PIN_ECHO_2, GPIO.IN)

      GPIO.output(PIN_TRIGGER_2, GPIO.LOW)

      #print ( "Waiting for sensor to settle")

      time.sleep(0.05)

      #print ( "Calculating distance")

      GPIO.output(PIN_TRIGGER_2, GPIO.HIGH)

      time.sleep(0.00001)

      GPIO.output(PIN_TRIGGER_2, GPIO.LOW)

      while GPIO.input(PIN_ECHO_2)==0:
            pulse2_start_time = time.time()
      while GPIO.input(PIN_ECHO_2)==1:
            pulse2_end_time = time.time()

      pulse2_duration = pulse2_end_time - pulse2_start_time
      distance2 = round(pulse2_duration * 17150, 2)
      #print ( "Distance of second sensor:",distance2,"cm")

      if distance2<(distance-30):
            people = people + 1
            time.sleep(1)
            print ( "Number of people:",people)

      elif distance<(distance2-30):
            people = people - 1
            if people<0:
                  people = 0
            print ( "Number of people:",people)
            time.sleep(1)

      GPIO.cleanup()