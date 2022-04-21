#!/usr/bin/env python

# 4/22/21
# INSTRUCTIONS

# Install this package: https://github.com/IntelRealSense/realsense-ros
# Instructions for using ROS bag:http://wiki.ros.org/rqt_bag
#
# Quick instructions:
# You must do this in your VM (which already has ROS melodic installed)
# In a terminal run following lines: 
#   source /opt/ros/melodic/setup.bash
#   rqt_bag 
# When rqt_bag interface pops up, open colored_bottles_image.bag
# Right click the camera/color/image_raw topic and click publish
# Hit green play button at top of window to play (this will start publishing this topic)

# Open new terminal and navigate to folder this script is located in and run

from __future__ import print_function

import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge, CvBridgeError
# from tkinter import *
# from tk import *
import numpy as np
import time
import math
import matplotlib.pyplot as plt

global depth_img, fine_requested, depth_ready, initial_location_ready, bridge

depth_img = np.zeros((550,640),dtype="uint8")
fine_requested = False
#depth_ready = True
#initial_location_ready = True

depth_ready = False
initial_location_ready = False

bridge = CvBridge()

#global canny, center, dist, d_p
#center=1
#dist=1000
#d_p=4
#canny=250

# Thresholds
red_low = np.array([0,0,0])
red_high = np.array([15,255,255])
green_low = np.array([80,0,0])
green_high = np.array([100,255,255])
blue_low = np.array([100,0,0])
blue_high = np.array([140,255,255])
yellow_low = np.array([20,0,0])
yellow_high = np.array([60,255,255])

def callback_depth(data):
  global depth_img, fine_requested, depth_ready, initial_location_ready, bridge

  if not depth_ready:
    try:
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        cv_image = cv_image[100:650, 640:]
        depth_array = np.array(cv_image, dtype=np.float32)
    except CvBridgeError as e:
      print(e)

    (rows,cols) = cv_image.shape
    print(rows)
    print(cols)
    #top_layer = np.argwhere(depth_array > 1000)
    #print(top_layer)

    depth_array = np.where(depth_array > 870, 0, 255)
    depth_img = depth_array.astype('uint8')
    depth_ready = True

    img = cv2.cvtColor(depth_img, cv2.COLOR_GRAY2BGR)
    kernel = np.ones((3,3), np.uint8)
    img_dilation = cv2.dilate(img, kernel, iterations=5)
    img_erosion = cv2.erode(img_dilation, kernel, iterations=2)

    edges = cv2.Canny(img,100,200)
    edge_dilate = cv2.dilate(edges, kernel, iterations=6)
    edge_dilate = cv2.bitwise_not(edge_dilate)
    minus_edges = cv2.bitwise_and(img_erosion,img_erosion,mask=edge_dilate)

    depth_img = minus_edges

    #TODO: Multiply depth mask to rgb image, restrict view to top level
    #TODO: Establish perimeter outside brick stack to ignore
    #TODO: Switch this topic to aligned with rgb
    #TODO: Establish height thresholds for different layers of bricks

    #print(max(top_layer[:,0]))

    #depth_array[top_layer] = 255

    #cv2.imshow("Depth Img", img)
    #cv2.imshow("filled holes", img_erosion)
    #cv2.waitKey(3)

    #plt.imshow(img_erosion,cmap='gray', vmin=0, vmax=255)
    plt.imshow(minus_edges,cmap='gray', vmin=0, vmax=255)
    plt.show()

    #pix = (data.width/2, data.height/2)
    #sys.stdout.write(str(cv_image[pix[1],pix[0]]))
    #time.sleep(5)

def callback_img(data):
  global depth_img, fine_requested, depth_ready, initial_location_ready, bridge

  if depth_ready and not initial_location_ready:
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        cv_image = cv_image[100:650, 640:]
        #cv_image = cv2.bitwise_and(cv_image,cv_image,mask=depth_img)
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    print(rows)
    print(cols)
    #print(rows)
    #print(cols)
    #cv_image[top_layer] = (0,0,0)
    cv2.imshow("Original_Image", cv_image)
    cv2.waitKey()

    ################ HSV THRESHOLDING ####################
    # convert to HSV
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # threshold
    red_mask_HSV = cv2.inRange(hsv_image, red_low, red_high)
    green_mask_HSV = cv2.inRange(hsv_image, green_low, green_high)
    blue_mask_HSV = cv2.inRange(hsv_image, blue_low, blue_high)
    yellow_mask_HSV = cv2.inRange(hsv_image, yellow_low, yellow_high)

    # get display image
    red_disp_image_HSV = cv2.bitwise_and(cv_image,cv_image, mask= red_mask_HSV)
    #cv2.imshow("RED HSV_Thresholding", red_disp_image_HSV)
    green_disp_image_HSV = cv2.bitwise_and(cv_image,cv_image, mask= green_mask_HSV)
    #cv2.imshow("GREEN HSV_Thresholding", green_disp_image_HSV)
    blue_disp_image_HSV = cv2.bitwise_and(cv_image,cv_image, mask= blue_mask_HSV)
    #cv2.imshow("BLUE HSV_Thresholding", blue_disp_image_HSV)
    yellow_disp_image_HSV = cv2.bitwise_and(cv_image,cv_image, mask= yellow_mask_HSV)

    cv2.imshow("YELLOW HSV_Thresholding", yellow_disp_image_HSV)
    cv2.waitKey()

    # convert the image to grayscale
    img = yellow_disp_image_HSV

    
    edges = cv2.Canny(cv_image,100,200)
    
    #edge_dilate = cv2.bitwise_not(edges)
    kernel = np.array([[0,1,0],[1,1,1],[0,1,0]]).astype('uint8')
    #img_dilation = cv2.dilate(img, kernel, iterations=5)
    #edges = cv2.Canny(img_dilation,100,200)
    img_dilate = cv2.erode(img.copy(), kernel, iterations=2)
    #edge_dilate = cv2.dilate(edges, kernel, iterations=1)
    #edge_dilate = cv2.bitwise_not(edge_dilate)
    #img = cv2.bitwise_and(img_dilation,img_dilation,mask=edge_dilate)
    #img = img_dilation
    
    # Detect points that form a line
    lines = cv2.HoughLinesP(edges,rho = 1,theta = 1*np.pi/180,threshold = 100,minLineLength = 120,maxLineGap = 5)
    # Draw lines on the image
    for line in lines:
      x1, y1, x2, y2 = line[0]
      xdiff = x2-x1
      ydiff = y2-y1
      cv2.line(img_dilate, (max(0,x1-xdiff/2), max(0,y1-ydiff/2)), (min(cols-1,x2+xdiff/2), min(cols-1,y2+ydiff/2)), (0, 0, 0), 3)

    img = img_dilate
    """
    for i in range(0, len(lines)):
      rho = lines[i][0][0]
      if rho>30:
        theta = lines[i][0][1]
        a = math.cos(theta)
        b = math.sin(theta)
        x0 = a * rho
        y0 = b * rho
        pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
        pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
        cv2.line(img, pt1, pt2, (0,0,0), 3, cv2.LINE_AA)
    """

    #img = cv2.dilate(img, kernel, iterations=5) dilating here breaks the mask (border??)
    gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    #edges = cv2.Canny(cv_image,100,200)
    #edges = cv2.bitwise_not(edges)
    #edge_dilate = cv2.dilate(edges, kernel, iterations=1)
    #edge_dilate = cv2.bitwise_not(edge_dilate)
    #img = cv2.bitwise_and(gray_image,gray_image,mask=edges)

    # convert the grayscale image toimg binary image
    ret,thresh = cv2.threshold(gray_image,20,255,0)
    cv2.imshow("Image", thresh)
    cv2.waitKey(0)

    # find contours in the binary image
    _, contours,hierarchy=cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(img,contours,-1,(0,255,0),3)

    for c in contours:
      # calculate moments for each contour
      M = cv2.moments(c)
      approx = cv2.approxPolyDP(c,0.01*cv2.arcLength(c,True),True)
      area = cv2.contourArea(c)

      # calculate x,y coordinate of center
      if M["m00"] != 0 and area > 10000 and area < 30000:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(img,[box],0,(0,0,255),2)
        print(area)
      else:
        cX, cY = 0, 0
      cv2.circle(img, (cX, cY), 5, (255, 255, 255), -1)
      cv2.putText(img, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # display the image
    cv2.imshow("Image", img)
    cv2.waitKey(0)

    initial_location_ready = True

    #TODO: Draw contours around rectangles
    #TODO: Choose desired brick + centroid
    #TODO: Transform & publish centroid+depth in image to robot frame target x,y,z


def callback(data):
  global depth_img, fine_requested, depth_ready, initial_location_ready, bridge


  try:
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
  except CvBridgeError as e:
    print(e)

  (rows,cols,channels) = cv_image.shape
  if cols > 60 and rows > 60 :
    cv2.circle(cv_image, (50,50), 10, 255)

  # cv2.imshow("Image window", cv_image)
  # cv2.waitKey(3)

  # visualize it in a cv window
  alignmentX=100 #The alignment dot's location, scaled for the smaller image
  alignmentY=62 #The alignment dot's location, scaled for the smaller image
  cv2.circle(cv_image,(alignmentX*4,alignmentY*4),2,(255,0,0),3)
  cv2.imshow("Original_Image", cv_image)
  cv2.waitKey(3)

  ################ HSV THRESHOLDING ####################
  # convert to HSV
  hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

  # get threshold values
  # lower_bound_HSV = np.array([l_h.get(), l_s.get(), l_v.get()])
  # upper_bound_HSV = np.array([u_h.get(), u_s.get(), u_v.get()])
  lower_bound_HSV = np.array([l_h, l_s, l_v])
  upper_bound_HSV = np.array([u_h, u_s, u_v])

  # threshold
  mask_HSV = cv2.inRange(hsv_image, lower_bound_HSV, upper_bound_HSV)

  # get display image
  disp_image_HSV = cv2.bitwise_and(cv_image,cv_image, mask= mask_HSV)
  cv2.imshow("HSV_Thresholding", disp_image_HSV)
  cv2.waitKey(3)
  time.sleep(0.05)

  ## Track circle
  # gray and blur
  resizeIm = cv2.resize(disp_image_HSV, (0,0), fx=0.25, fy=0.25)
  blurIm = cv2.medianBlur(resizeIm,5)
  grayIm = cv2.cvtColor(blurIm,cv2.COLOR_BGR2GRAY) # Convert to grascale image

  # hough
  #TODO: Play with these values
  arg1 = canny#.get()     #Upper threshold for Canny edge detection
  arg2 = center#.get()       #Threshold for center detection. 
  min_distance = dist#.get()    # Minimum distance between new circle centers. 
  dp = d_p#.get()             # How accepting to degradation of circles are you willing to be
  circles = cv2.HoughCircles(grayIm,cv2.HOUGH_GRADIENT,dp,min_distance,param1=arg1,param2=arg2, minRadius=0,maxRadius=60)
  offset=10
  if circles is not None:
      circles = np.uint16(np.around(circles,0))
      for i in circles[0,:]:
          # draw circle
          cv2.circle(disp_image_HSV,(4*i[0],4*i[1]),4*i[2],(0,255,0),2)
          # cv2.circle(disp_image_HSV,(4*i[0]-4*offset,4*i[1]),2,(0,0,255),3) 
          # cv2.circle(disp_image_HSV,(4*alignmentX,4*alignmentY),2,(255,0,0),3)               
          # print("Object Radius (pxls): ",i[2],"Object Offset (pxls): ",math.ceil(1.1*i[2]))
            
  cv2.imshow("Hough Circle Detection", disp_image_HSV)
  cv2.waitKey(3)

def callback_fine_request(data):
  global fine_requested
  fine_requested = True

def callback_coarse_request(data):
  global depth_ready, initial_location_ready
  depth_ready = False
  initial_location_ready = False

def listener():
    coarse_request_sub = rospy.Subscriber("/coarse_request",Bool, callback_coarse_request)
    #fine_request_sub = rospy.Subscriber("/fine_request",Bool, callback_fine_request)
    image_sub = rospy.Subscriber("/cam_1/color/image_raw",Image, callback_img)
    depth_sub = rospy.Subscriber("/cam_1/aligned_depth_to_color/image_raw", Image,callback=callback_depth, queue_size=1)
    brick_location = rospy.Publisher("/brick_location",Pose,queue_size=1)
    #image_pub = rospy.Publisher("image_topic_2",Image)

def main(args):
   
    rospy.init_node('image_converter', anonymous=True)
    
    listener()

    try:
      rospy.spin()
    except KeyboardInterrupt:
      print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


