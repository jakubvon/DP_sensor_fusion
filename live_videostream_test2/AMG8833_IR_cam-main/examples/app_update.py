from flask import Flask,render_template,Response
import time,sys
import random
import json
sys.path.append('../')
import amg8833_i2c
import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate
import cv2
import RPi.GPIO as GPIO
from array import *

app =Flask(__name__)

#define important variables
print ( "Number of people: 0")
motion_detect = "idk"

## pin setup

#ultrasonic pins
pin_trigger = array('i',[16,13])          #trigger pins on RPi of both sensors
pin_echo = array('i',[18,15])             #echo pins on RPi of both sensors

PIR_PIN = 7
radar_PIN = 11


## setup pins and board mode

GPIO.setmode(GPIO.BOARD)

#ultrasonic pins setup
GPIO.setup(pin_trigger[0], GPIO.OUT)
GPIO.setup(pin_echo[0], GPIO.IN)
GPIO.setup(pin_trigger[1], GPIO.OUT)
GPIO.setup(pin_echo[1], GPIO.IN)

GPIO.setup(radar_PIN,GPIO.IN)           #radar pin setup
GPIO.setup(PIR_PIN,GPIO.IN)             #PIR pin setup

#####################################
# Initialization of Grid EYE Sensor
#####################################
#
t0 = time.time()
sensor = []
while (time.time()-t0)<1: # wait 1sec for sensor to start
    try:
        # AD0 = GND, addr = 0x68 | AD0 = 5V, addr = 0x69
        sensor = amg8833_i2c.AMG8833(addr=0x69) # start AMG8833
    except:
        sensor = amg8833_i2c.AMG8833(addr=0x68)
    finally:
        pass
time.sleep(0.1) # wait for sensor to settle

# If no device is found, exit the script
if sensor==[]:
    print("No AMG8833 Found - Check Your Wiring")
    sys.exit(); # exit the app if AMG88xx is not found 
#

#####################################
# Interpolation Properties 
#####################################
#
# original resolution
pix_res = (8,8) # pixel resolution
xx,yy = (np.linspace(0,pix_res[0],pix_res[0]),
                    np.linspace(0,pix_res[1],pix_res[1]))
zz = np.zeros(pix_res) # set array with zeros first
# new resolution
pix_mult = 6 # multiplier for interpolation 
interp_res = (int(pix_mult*pix_res[0]),int(pix_mult*pix_res[1]))
grid_x,grid_y = (np.linspace(0,pix_res[0],interp_res[0]),
                            np.linspace(0,pix_res[1],interp_res[1]))
# interp function
def interp(z_var):
    # cubic interpolation on the image
    # at a resolution of (pix_mult*8 x pix_mult*8)
    f = interpolate.interp2d(xx,yy,z_var,kind='cubic')
    return f(grid_x,grid_y)
grid_z = interp(zz) # interpolated image

#
#####################################
# Start and Format Figure 
#####################################
#
plt.rcParams.update({'font.size':16})
fig_dims = (10,9) # figure size
fig,ax = plt.subplots(figsize=fig_dims) # start figure
fig.canvas.manager.set_window_title('AMG8833 Image Interpolation')
im1 = ax.imshow(grid_z,vmin=18,vmax=37,cmap=plt.cm.RdBu_r) # plot image, with temperature bounds
cbar = fig.colorbar(im1,fraction=0.0475,pad=0.03) # colorbar
cbar.set_label('Temperature [C]',labelpad=10) # temp. label
fig.canvas.draw() # draw figure

ax_bgnd = fig.canvas.copy_from_bbox(ax.bbox) # background for speeding up runs
fig.show() # show figure
#

#####################################
# Plot AMG8833 temps in real-time
#####################################
#
pix_to_read = 64 # read all 64 pixels

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

if distance1_initial>130:
    distance1_initial=130

if distance2_initial>130:
    distance2_initial=130


def generate_frames():
    last_movement = time.time() - 10
    order = "3"                               #3 - default, 12 - someone stepped in, 21-someone stepped out, other - not ok ->reset
    timeout = 0                               #timeout - used so it doesnt count people when only standing near door
    people = 0                                #number of people in the room counted
    global people_termal
    global is_motion_found
    people_termal = 0
    while True:
      
        # measure distances by both sensors and round down if necessary

      distance1 = measureDistance(0)
      distance2 = measureDistance(1)

      if distance1>130:
            distance1=130

      if distance2>130:
            distance2=130


      if distance1<(distance1_initial-30) and order[0] != "1": #if change in distance by sensor1 and it hasnt been already reported in previous rounds => write 1 in the sequence first place
                if order == "3":
                    order = ""
                order += "1"

      elif distance2<(distance2_initial-30) and order[0] != "2": #if change in distance by sensor1 and it hasnt been already reported in previous rounds => write 2 in the sequence first place
                if order == "3":
                    order = ""
                order += "2"

      if order == "12":                                     #first activated sensor1,then sensor2 -> someone came in the room
                if timeout<5:                                  # makes sure that noone is just standing in front of the sensors
                    order="3"                                 # change sequence to default
                else:
                    people = people + 1                       # counts +1 person
                    order = "3"                               # change sequence to default
                    print ( "Number of people:",people) 
                         
                    time.sleep(1)                             # waits till he is gone

      elif order == "21" and people > 0:                    #first activated sensor2,then sensor1 -> someone left the room (+cant be less than 0)
                if timeout<5:
                    order = "3"
                else:
                    people = people - 1                       # counts -1 person
                    order = "3"
                    print ( "Number of people:",people)
                    
                    time.sleep(1)

      elif order == "21" and people == 0:                   #cant be less than 0 -> reset sequence, still 0 people
                order = "3"

      if len(order)>2 or order == "11" or order == "22" or timeout>1000:            #fault sequences or timeout => reset
                order = "3"
                timeout = 0        

      if len(order)==1 and order!="3":                                        #if one sensor detected, start timeout count
                timeout = timeout+1
      else:
                timeout = 0                                                       #if sequence alreadz reset => reset timeout


      time_now = time.time()
      if GPIO.input(radar_PIN) or GPIO.input(PIR_PIN):
          is_motion_found = "yes"
          last_movement = time.time()
      if (time_now-last_movement)<3: 
        status,pixels = sensor.read_temp(pix_to_read) # read pixels with status
        if status: # if error in pixel, re-enter loop and try again
            continue
        
        T_thermistor = sensor.read_thermistor() # read thermistor temp
        fig.canvas.restore_region(ax_bgnd) # restore background (speeds up run)
        new_z = interp(np.reshape(pixels,pix_res)) # interpolated image

        #print(new_z)
        plt.imsave('color_img.jpg', new_z)
        
        img = cv2.imread("color_img.jpg", cv2.IMREAD_GRAYSCALE)
        img = cv2.bitwise_not(img)
    # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()

            # Change thresholds
        params.minThreshold = 40 # 10
        params.maxThreshold = 255

            # Filter by Area.
        params.filterByArea = True
        params.minArea = 100

            # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.1 # 0.1

            # Filter by Convexity
        params.filterByConvexity = False
        params.minConvexity = 0.9 #0.87

            # Filter by Inertia
        params.filterByInertia = False
        params.minInertiaRatio = 0.01

            # Set up the detector with default parameters.
        detector = cv2.SimpleBlobDetector_create(params)

            # Detect blobs.
        keypoints = detector.detect(img)

        im_with_keypoints = cv2.drawKeypoints(img,keypoints,np.array([]),(0,0,255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        im1.set_data(im_with_keypoints) # update plot with new interpolated temps
        ax.draw_artist(im1) # draw image again
        fig.canvas.blit(ax.bbox) # blitting - for speeding up run
        fig.canvas.flush_events() # for real-time plot

        people_termal = len(keypoints)

        for i in range (0, len(keypoints)):
            x = keypoints[i].pt[0]
            y = keypoints[i].pt[1]
            #print(x, y)
            #print("number of people is: ", (len(keypoints)))
            if people_termal>0:
                last_movement = time.time()
        

        ret,buffer = cv2.imencode('.png',im_with_keypoints)
        im_with_keypoints=buffer.tobytes()

        #yield (str(people_termal) + '\n')

        yield(b'--im_with_keypoints\r\n'
                   b'Content-Type: image/png\r\n\r\n' + im_with_keypoints + b'\r\n')
        

      else:
          is_motion_found = "no"
          print("no movement bro")

          # prepnout kameru do sleepmode by bylo fajn

# here needs to be the camera frames loaded


@app.route('/')
def index():
    return render_template('index.html')

@app.route('/number_of_people')
def number_of_people():
    #number_of_people = random.randint(1,10)
    number_of_people = people_termal
    return json.dumps({'number_of_people': number_of_people})

@app.route('/motion_found')
def motion_found():
    #number_of_people = random.randint(1,10)
    motion_found = is_motion_found
    return json.dumps({'motion_found': motion_found})

@app.route('/video')
def video():
    return Response(generate_frames(),mimetype='multipart/x-mixed-replace; boundary=im_with_keypoints')

if __name__=="__main__":
    app.run(debug=True, port=80, host='192.168.0.10')