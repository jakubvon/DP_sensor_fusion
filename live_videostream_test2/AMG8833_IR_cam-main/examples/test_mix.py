#######################################################
# Thermal camera Plotter with AMG8833 Infrared Array
# --- with interpolation routines for smoothing image
#
# by Joshua Hrisko
#    Copyright 2021 | Maker Portal LLC
#
#######################################################
#
import time,sys
sys.path.append('../')
# load AMG8833 module
import amg8833_i2c
import numpy as np
import math
import matplotlib.pyplot as plt
from scipy import interpolate
from scipy.interpolate import griddata
import cv2
#
#####################################
# Initialization of Sensor
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
active = True
pix_to_read = 64 # read all 64 pixels

while(1):
    if active == True:
        # original resolution
        pix_res = (8,8) # pixel resolution
        xx,yy = (np.linspace(0,pix_res[0],pix_res[0]),
                    np.linspace(0,pix_res[1],pix_res[1]))
        zz = np.zeros(pix_res) # set array with zeros first
        # new resolution
        pix_mult = 32  # multiplier for interpolation 
        interp_res = (int(pix_mult*pix_res[0]),int(pix_mult*pix_res[1]))
        grid_x,grid_y = (np.linspace(0,pix_res[0],interp_res[0]),
                            np.linspace(0,pix_res[1],interp_res[1]))
# interp function
        # bicubic interpolation of 8x8 grid to make a 32x32 grid
        status,pixels = sensor.read_temp(pix_to_read) # read pixels with status
        points = [(math.floor(ix / 8), (ix % 8)) for ix in range(0, 64)]
        bicubic = griddata(points, pixels, (grid_x, grid_y), method='cubic')
        image = np.array(bicubic)
        image = np.reshape(image, (16, 16))
        print(image)
        plt.imsave('color_img.jpg', image)

        # Read image
        img = cv2.imread("color_img.jpg", cv2.IMREAD_GRAYSCALE)
        img = cv2.bitwise_not(img)

        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()

        # Change thresholds
        params.minThreshold = 10
        params.maxThreshold = 255

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 5

        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.1

        # Filter by Convexity
        params.filterByConvexity = False
        params.minConvexity = 0.87

        # Filter by Inertia
        params.filterByInertia = False
        params.minInertiaRatio = 0.01

        # Set up the detector with default parameters.
        detector = cv2.SimpleBlobDetector_create(params)

        # Detect blobs.
        keypoints = detector.detect(img)

        im_with_keypoints = cv2.drawKeypoints(img,keypoints,np.array([]),(0,0,255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        

        for i in range (0, len(keypoints)):
            x = keypoints[i].pt[0]
            y = keypoints[i].pt[1]
            print(x, y)