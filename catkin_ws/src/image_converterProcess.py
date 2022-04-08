        

#!/usr/bin/env python
from __future__ import print_function

import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# from tkinter import *
# from tk import *
import numpy as np
import time
import math

class image_converter:

  def __init__(self):
    self.bridge = CvBridge()
    # self.image_sub = rospy.Subscriber("image_topic",Image,self.callback)
    # self.image_pub = rospy.Publisher("image_topic_2",Image)
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)

    # self.image_sub = rospy.Subscriber("/camera/infra1/image_rect_raw",Image,self.callback)
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    # rospy.init_node('image_processing', anonymous=True)
    # rospy.Subscriber("/camera/depth/image_rect_raw/", Image, self.depth_cb)
    # global tk
    # tk = Tk()

    # # canny = Scale(tk, from_ = 1, to = 1500, label = 'Upper threshold', orient = HORIZONTAL)
    # # canny.pack()
    # # Edit default value here e.g. canny.set(##)
    # canny=250
    # # center = Scale(tk, from_ = 1, to = 1000, label = 'Center Threshold', orient = HORIZONTAL)
    # # center.pack()
    # # # Edit default value here
    # center=1
    # # dist = Scale(tk, from_ = 1, to = 1000, label = 'Min Centers dist', orient = HORIZONTAL)
    # # dist.pack()
    # # Edit default value here
    # dist=1000
    # # d_p = Scale(tk, from_ = 0.1, to = 5, label = 'Accumulator Ratio', orient = HORIZONTAL, resolution = 0.01)
    # # d_p.pack()
    # # Edit default value here
    # d_p=4
    # # global l_h, u_h, l_s, u_s, l_v, u_v
    # # l_h = Scale(tk, from_ = 0, to = 255, label = 'Hue, lower', orient = HORIZONTAL)
    # # l_h.pack()
    # # Edit default value here
    # l_h=174
    # # u_h = Scale(tk, from_ = 0, to = 255, label = 'Hue, upper', orient = HORIZONTAL)
    # # u_h.pack()
    # # Edit default value here
    # u_h=199
    # # l_s = Scale(tk, from_ = 0, to = 255, label = 'Saturation, lower', orient = HORIZONTAL)
    # # l_s.pack()
    # # Edit default value here
    # l_s=71
    # # u_s = Scale(tk, from_ = 0, to = 255, label = 'Saturation, upper', orient = HORIZONTAL)
    # # u_s.pack()
    # # Edit default value here
    # u_s=255
    # # l_v = Scale(tk, from_ = 0, to = 255, label = 'Value, lower', orient = HORIZONTAL)
    # # l_v.pack()
    # # Edit default value here
    # l_v=0
    # # u_v = Scale(tk, from_ = 0, to = 255, label = 'Value, upper', orient = HORIZONTAL)
    # # u_v.pack()
    # # Edit default value here
    # u_v=255

    # l_h = 174
    # u_h = 199
    # l_s = 71
    # u_s = 255 
    # l_v = 0
    # u_v = 255

    # self.bridge = CvBridge()
    # # self.image_sub = rospy.Subscriber("image_topic",Image,self.callback)
    # # self.image_pub = rospy.Publisher("image_topic_2",Image)

    # self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)

    # # self.image_sub = rospy.Subscriber("/camera/infra1/image_rect_raw",Image,self.callback)
    # self.image_pub = rospy.Publisher("image_topic_2",Image)

  def callback(self,data):


    try:
      # cv_image = self.bridge.imgmsg_to_cv2(image_sub, "bgr8")
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
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
            cv2.circle(disp_image_HSV,(4*i[0]-4*offset,4*i[1]),2,(0,0,255),3) 
            cv2.circle(disp_image_HSV,(4*alignmentX,4*alignmentY),2,(255,0,0),3)               
            print("Object Radius (pxls): ",i[2],"Object Offset (pxls): ",math.ceil(1.1*i[2]))

    cv2.imshow("Hough Circle Detection", disp_image_HSV)
    cv2.waitKey(3)

    # try:
    #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    # except CvBridgeError as e:
    #   print(e)

def main(args):
    ic = image_converter()
    global canny, center, dist, d_p
    global l_h, u_h, l_s, u_s, l_v, u_v
    center=1
    dist=1000
    d_p=4
    canny=250

    l_h = 174
    u_h = 199
    l_s = 71
    u_s = 255 
    l_v = 0
    u_v = 255

    rospy.init_node('image_converter', anonymous=True)

    try:
      rospy.spin()
    except KeyboardInterrupt:
      print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


