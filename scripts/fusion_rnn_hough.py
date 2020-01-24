#!/usr/bin/env python
## Author: Alan Tavares
## Date: August, 12, 2019
# Purpose: Ros node to detect objects using tensorflow
import os
import sys
import cv2
import numpy as np
import math

# ROS related imports
import rospy
from geometry_msgs.msg import Twist, Vector3

# Navigation
class Navigation:
 
  def __init__(self):

        self.VERBOSE = True
        self.msg_hough = Twist()
        
        #############################################################################
        # Publishers
        self.nav_pub = rospy.Publisher('navigation/nav_position', Twist, queue_size=10)

        # Subscribers
        self.hough_sub = rospy.Subscriber("hough/nav_hough_lines", Twist, self.houghCallback, queue_size=10)
        self.rnn_sub = rospy.Subscriber("RNN/nav_direction", Vector3, self.rcnnCallback, queue_size=10)

  #############################################################################
        
  def houghCallback(self, data):
        # recive data
        self.msg_hough = data

  #############################################################################
        
  def rcnnCallback(self, data):
        # recive data
        moviment = data.x
        rotation = data.y

        velocity = Twist()

        # controll y and yaw only
        if moviment == 1:
            if self.VERBOSE == True:
                rospy.logdebug("Curva")

            if rotation == 1:
                velocity.linear.x = 0
                velocity.linear.y = 0
                velocity.linear.z = 0

                velocity.angular.x = 0
                velocity.angular.y = 0
                velocity.angular.z = math.radians(8)
                if self.VERBOSE == True:
                    rospy.logdebug("...Left-yaw: %f deg/s",velocity.angular.z*(180/np.pi))
                    rospy.logdebug("-------------------------")

            else:
                velocity.linear.x = 0
                velocity.linear.y = 0
                velocity.linear.z = 0

                velocity.angular.x = 0
                velocity.angular.y = 0
                velocity.angular.z = math.radians(-8)
                if self.VERBOSE == True:
                    rospy.logdebug("...Right-yaw: %f deg/s",velocity.angular.z*(180/np.pi))
                    rospy.logdebug("-------------------------")
        else:
            if self.VERBOSE == True:
                rospy.logdebug("Reta")
            velocity.linear.x = 0
            velocity.linear.y = -self.msg_hough.linear.y
            velocity.linear.z = 0

            velocity.angular.x = 0
            velocity.angular.y = 0
            velocity.angular.z = self.msg_hough.angular.z
            if self.VERBOSE == True:
                rospy.logdebug('Y-raw  : %f', velocity.linear.y)
                rospy.logdebug('Yaw-raw: %f', math.degrees(velocity.angular.z))
                rospy.logdebug("-------------------------")

        self.nav_pub.publish(velocity)

###############################################################################

def main(args):
  #-- Name of node
  rospy.init_node('navigation_node',log_level=rospy.DEBUG)
  ic = Navigation()

  try:
      rospy.spin()
  except KeyboardInterrupt:
      print("Shutting down")

###############################################################################
   
if __name__ == '__main__':
  main(sys.argv)