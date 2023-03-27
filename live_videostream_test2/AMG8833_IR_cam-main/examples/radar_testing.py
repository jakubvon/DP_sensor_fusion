import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
radar_PIN = 11
GPIO.setup(radar_PIN,GPIO.IN)
PIR_PIN = 7
GPIO.setup(PIR_PIN,GPIO.IN)

#done till here

while 1:

    if GPIO.input(radar_PIN) and GPIO.input(PIR_PIN):
        print("ya moved bitches")
        time.sleep(1)

    elif GPIO.input(radar_PIN):
        print("ya moved bitch")
        time.sleep(1)
    elif GPIO.input(PIR_PIN):
        print("ya moved")
        time.sleep(1)
    

#def MOTION(radar_PIN):
 #   print("ya moved bitch")

#try:
 #   GPIO.add_event_detect(radar_PIN,GPIO.RISING,callback=MOTION)
  #  while 1:
   #     time.sleep(0.1)
#except KeyboardInterrupt:
    else:
        print("bitch")
        time.sleep(1)
    
GPIO.cleanup()