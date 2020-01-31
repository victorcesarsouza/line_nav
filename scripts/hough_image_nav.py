#!/usr/bin/env python
from __future__ import print_function
## Author: Alan Tavares
## Date: August, 12, 2019
# Purpose: Ros node to detect objects using tensorflow
import os
import sys, time, math
import cv2
import numpy as np
import rospy
import roslib
roslib.load_manifest('line_nav')

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

class hough_lines:
 
  def __init__(self):
    #-- Create a publisher in topic "image_hough" and "nav_hough_lines"
    self.nav_hough_lines_pub = rospy.Publisher("hough/nav_hough_lines",Twist, queue_size = 1)
    
    #self.image_raw_pub = rospy.Publisher("hough/image_raw/compressed", CompressedImage, queue_size = 1)
    self.image_hough_pub = rospy.Publisher("hough/image_hough", Image, queue_size = 1)    
    self.image_edge_pub = rospy.Publisher("hough/image_edge", Image, queue_size = 1)

    #-- Create a supscriber from topic "image_raw"
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("bebop/image_raw", Image, self.callback, queue_size = 1)
    
    # self.list_hough = []
    self.ky = 52

    self.MIN_EDGE = rospy.get_param('~min_edge', 450)   #470 sol  - 450 MELHOR  - 480  --
    self.MAX_EDGE = rospy.get_param('~max_edge', 500)   #520 sol  - 500 MELHOR  - 530 --

    rospy.loginfo("%s is %f (defaut)", rospy.resolve_name('~min_edge'), self.MIN_EDGE)
    rospy.loginfo("%s is %f (defaut)", rospy.resolve_name('~max_edge'), self.MAX_EDGE)

###############################################################################
   
  def callback(self,data):

    numLines=3
    yaw = 0
    x = 0
    med_theta = 0
    lines_vector = [0, 0, 0]

    try:
        image_np = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
    # image_np = cv2.cvtColor(cv_image,cv2.COLOR_BGR2RGB)

    # (rows,cols,channels) = image_np.shape
    # rospy.logdebug("rows: %f",rows)
    # rospy.logdebug("cols: %f",cols)
    # rospy.logdebug("-------------------------")

    #-- Resize image with INTER_CUBIC
    resize = cv2.resize(image_np, (224, 224), interpolation=cv2.INTER_CUBIC)

    #-- Convert in gray scale
    gray = cv2.cvtColor(resize, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red

    #-- Detection de edges
    edges = cv2.Canny(gray, self.MIN_EDGE, self.MAX_EDGE, apertureSize=3, L2gradient=True) # default (350,400)

    #-- Blur bilateral filter
    blur = cv2.bilateralFilter(edges,3,75,75)
    #blur2 = cv2.GaussianBlur(edges,(5,5),0)

    #-- Erosion and Dilation
    kernel_dil = np.ones((5,5), np.uint8)
    kernel_ero = np.ones((3,3), np.uint8)

    dilation = cv2.dilate(blur, kernel_dil, iterations=1)
    erosion = cv2.erode(dilation, kernel_ero, iterations=1) 


    # Otsu's thresholding after Gaussian filtering
    #gray2 = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red
  
    #ret1, th1 = cv2.threshold(gray2,100,150,cv2.THRESH_BINARY)
    #ret2, th2 = cv2.threshold(gray2,50,100,cv2.THRESH_OTSU)
    #blur2 = cv2.GaussianBlur(gray2,(5,5),0)
    #ret3, th3 = cv2.threshold(blur2,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

    #Deteccao de linhas
    lines = cv2.HoughLines(erosion, numLines, np.pi/90, 100)

    if lines is not None: 
            if lines.shape[0] >= numLines:
                x = 0
                med_theta = 0
                for i in range(0,numLines):
                    for rho, theta in lines[i]:
                        a = np.cos(theta)
                        b = np.sin(theta)
                        x0 = a*rho
                        y0 = b*rho
                        x1 = int(x0 + 1000*(-b))
                        y1 = int(y0 + 1000*(a))
                        x2 = int(x0 - 1000*(-b))
                        y2 = int(y0 - 1000*(a))
   
                        cv2.line(resize, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        
                        med_theta = med_theta + (theta/numLines)
                        lines_vector[i] = theta
                        x = x+x1+x2


    mediana = int(x/(numLines*2))

    med_theta = math.degrees(med_theta)
    
    # zerar erro de leitura Yaw
    if abs( math.degrees(lines_vector[0]) - math.degrees(lines_vector[1]) ) < 60 and abs( math.degrees(lines_vector[0]) - math.degrees(lines_vector[2]) ) < 60 and abs( math.degrees(lines_vector[1]) - math.degrees(lines_vector[2]) ) < 60:
      if med_theta > (90):
        yaw = (180-med_theta)
      else:
        yaw = -med_theta

    # rospy.logdebug("linha 1: %f",math.degrees(lines_vector[0]))
    # rospy.logdebug("linha 2: %f",math.degrees(lines_vector[1]))
    # rospy.logdebug("linha 3: %f",math.degrees(lines_vector[2]))

    # rospy.logdebug("Media Theta: %f",med_theta)
    # rospy.logdebug("Valor x: %f",x)
    # rospy.logdebug("-------------------------")

    # y in the drone of ROS = X in the image
    y_correction = float(mediana - resize.shape[1]/2)/self.ky

    #rospy.logdebug("half_img: %f",erosion.shape[1]/2)
    #rospy.logdebug("mediana: %f",mediana)
    ####################################################
    # covert y_correction in Yreal

    # rospy.logdebug("y(raw): %f",y_correction)
    # Zreal = 0.31
    # Ynoparameter = y_correction
    # Yreal = 0.28
    # k = (Ynoparameter*Zreal)/Yreal
    # rospy.logdebug("Ky: %f",k)
    # ky = 52
    # Yreal = (Ynoparameter*Zreal)/ky
    # rospy.logdebug("y(Real): %f (m)", Yreal)
    # rospy.logdebug("yaw(raw): %f",yaw)
    # rospy.logdebug("-------------------------")
    ####################################################

    # Filter yaw
    # if len(self.list_hough) < 1:
    #     self.list_hough.append(yaw)
        
    # else:
    #     self.list_hough.append(yaw)
    #     del self.list_hough[0]
    #     yaw_filtered = sum(self.list_hough)/len(self.list_hough)
    #     rospy.logdebug('Size of list_hough %f',len(self.list_hough))
    #     rospy.logdebug('Yaw (Filter): %f deg/s',yaw_filtered)
    #     rospy.logdebug("-------------------------")


    nav_drone = Twist()

    if lines is not None:
      nav_drone.linear.x = 0
      nav_drone.linear.y = y_correction
      nav_drone.linear.z = 0

      nav_drone.angular.x = 0
      nav_drone.angular.y = 0
      nav_drone.angular.z = math.radians(yaw)
    else:
      nav_drone.linear.x = 0
      nav_drone.linear.y = 0
      nav_drone.linear.z = 0

      nav_drone.angular.x = 0
      nav_drone.angular.y = 0
      nav_drone.angular.z = 0

    # rospy.logdebug('Yaw Raw: %f deg/s',yaw)
    # rospy.logdebug("-------------------------")

    try:
      self.nav_hough_lines_pub.publish(nav_drone)
      #rospy.logdebug('Is publish!')
    except:
      rospy.logdebug('No publish lines!')

    #cv2.imshow("Image",src_image)
    #cv2.imshow("Image-edges",edges)
    
    #cv2.imshow("Image-blur",blur)

    #cv2.imshow("Image-dilation",dilation)
    #cv2.imshow("Image-erosion",erosion)
    #cv2.waitKey(1)

    try:
        self.image_hough_pub.publish(self.bridge.cv2_to_imgmsg(resize, "bgr8"))
        self.image_edge_pub.publish(self.bridge.cv2_to_imgmsg(erosion, "mono8"))
    except CvBridgeError as e:
        print(e)
        rospy.logdebug('No publish img!')

###############################################################################

def main(args):

  ic = hough_lines()
  #-- Name of node
  rospy.init_node('hough_node',log_level=rospy.DEBUG)

  try:
      rospy.spin()
  except KeyboardInterrupt:
      print("Shutting down")

  #cv2.destroyAllWindows()

###############################################################################
   
if __name__ == '__main__':
  main(sys.argv)
