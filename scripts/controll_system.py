#!/usr/bin/env python
## Author: Alan Tavares
## Date: January, 23, 2020
from __future__ import print_function
 
import roslib
roslib.load_manifest('line_nav')

import time, math
import sys, select, termios, tty
import rospy
import tf
import cv2

# numpy and scipy
import numpy as np
from std_msgs.msg import Empty
from std_msgs.msg import String
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

vel_hough = Twist()
pose_rcnn = Vector3()

msg = """
Keyboard commands for Autonomous Landing of the Quadcopter
----------------------------------------------------------
TakeOff         - Press 1
Landing         - Press 2
MoveCamera      - Press 3
Auto-Navigation - Press 4
----------------------------------------------------------
Ctrl + C to quit
"""

###############################################################################

def getKey():
  tty.setraw(sys.stdin.fileno())
  select.select([sys.stdin], [], [], 0)
  key = sys.stdin.read(1)
  termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
  return key

###############################################################################

def moveCamera():

  cam_twist = Twist()

  #-- set camera, look to dwn
  cam_twist.angular.x = 0
  cam_twist.angular.y = -84
  cam_twist.angular.z = 0
  cam_pub.publish(cam_twist)
  rospy.loginfo('angle_camera: %f',cam_twist.angular.y)

  return cam_twist.angular.y

###############################################################################

def autoNavigation():
  global vel_hough, pose_rcnn

  last_error_x = 0
  last_error_y = 0
  last_error_z = 0
  last_error_yaw = 0

  int_error_x = 0
  int_error_y = 0
  int_error_z = 0
  int_error_yaw = 0
  
  last_x = 0.02
  Vref = 0.02
  Vcx = 0

  init = True
  navigation = True
  Keyframe = 0

  current_x = 0

  current_time = rospy.Time.now()
  last_time = rospy.Time.now()

  while not rospy.is_shutdown() and navigation == True:

    current_time = rospy.Time.now()
    # compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).to_sec()

    #######################################################################
    
    #Gain controll X
    Kpx = 0.5  #0.07
    Kix = 0.05  #0.0001
    Kdx = 0.005  #0.5
    #Erro of Vx
    if(pose_rcnn.x != 0):
        current_x = pose_rcnn.x
        # rospy.loginfo('current_x ok!')
    else:
        current_x = Vref
        Vcx = 0.02
        # rospy.loginfo('current_x Nonve! Call Vref')
    # rospy.loginfo('current_x: %f',current_x)

    if init == False and dt > 0 and Vcx != Vref:
        Vcx = float((current_x-last_x)/dt)
        # rospy.loginfo('Vcx: %f and dt %f',Vcx, dt)

    erro_vx = (Vref - Vcx)
    # rospy.loginfo('erro_vx: %f',erro_vx)

    if abs(erro_vx) > 0.005:
        new_x = Kpx*erro_vx + Kix*int_error_x + Kdx*(erro_vx-last_error_x)
        int_error_x += erro_vx
        last_error_x = erro_vx
        #rospy.loginfo('new_x: %f',new_x)
        #rospy.loginfo("-------------------------")
    else:
        new_x = 0.02
        int_error_x = 0
        last_error_x = 0
        #rospy.loginfo('new_x: %f',new_x)
        #rospy.loginfo("-------------------------")
    last_x = current_x
    # rospy.loginfo('last_x: %f',last_x)

    #######################################################################
    #Gain controll Y
    Kpy = 0.01  #0.07
    Kiy = 0.0005  #0.0001
    Kdy = 0.00005  #0.5
    #Erro of Y
    y_raw = vel_hough.linear.y
    # rospy.loginfo('y_raw: %f',y_raw)
    erro_y = y_raw
    if abs(erro_y) > 0.1:
        new_y = Kpy*erro_y + Kiy*int_error_y + Kdy*(erro_y-last_error_y)
        int_error_y += erro_y
        last_error_y = erro_y
        #rospy.loginfo('new_y: %f',new_y)
        #rospy.loginfo("-------------------------")
    else:
        new_y = 0
        int_error_y = 0
        last_error_y = 0
        #rospy.loginfo('new_y: %f',new_y)
        #rospy.loginfo("-------------------------")
    #######################################################################
    # Gain controll Z
    kpz = 0.1
    Kiz = 0.001
    Kdz = 0.0001
    set_point = 1.0
    # Recive pose Z
    if(pose_rcnn.z != 0):
        z_raw = pose_rcnn.z
        # rospy.loginfo('z_raw ok!')
    else:
        z_raw = set_point
        # rospy.loginfo('z_raw Nonve! Call set_point')
    # rospy.logdebug('z_raw: %f',z_raw)
    #Erro of Z
    erro_z = float(set_point - z_raw)
    # rospy.loginfo("erro_z %f", erro_z)

    if abs(erro_z) > 0.1:
        new_z = kpz*erro_z + Kiz*int_error_z + Kdz*(erro_z-last_error_z)
        int_error_z += erro_z
        last_error_z = erro_z
        #rospy.loginfo("Correction Z %f", new_z)
        #rospy.loginfo("-------------------------")
    else:
        new_z = 0
        int_error_z = 0
        last_error_z = 0
        #rospy.loginfo("Correction Z %f", new_z)
        #rospy.loginfo("-------------------------")
    #######################################################################
    #Gain controll Yaw
    Kp = 0.6  #0.6
    Ki = 0.001  #0.001
    Kd = 0.0001  #0.0001
    #Erro of Yaw
    yaw_raw = math.degrees(vel_hough.angular.z)
    # rospy.loginfo('yaw_raw: %f',yaw_raw)
    erro_yaw = yaw_raw

    if abs(erro_yaw) > 4:
        new_yaw = Kp*erro_yaw #+ Ki*int_error_yaw #+ Kd*(erro_yaw-last_error_yaw)
        int_error_yaw += erro_yaw
        last_error_yaw = erro_yaw
        # rospy.loginfo('new_yaw: %f',new_yaw)
        # rospy.loginfo("-------------------------")
    else:
        new_yaw = 0
        int_error_yaw = 0
        last_error1 = 0
        # rospy.loginfo('new_yaw: %f',new_yaw)
        # rospy.loginfo("-------------------------")
    
    velocity = Twist()

    if init == True:
      for i in range(50):
        velocity = Twist()
        velocity.linear.y = -0.02
        velocity.linear.z = 2
        vel_drone_pub.publish(velocity)
        rospy.loginfo('Z: SUBINDO!')
        rate.sleep()

    init = False

    # rospy.loginfo("Navigation")
    velocity.linear.x = new_x
    velocity.linear.y = -new_y
    velocity.linear.z = new_z

    velocity.angular.x = 0
    velocity.angular.y = 0
    velocity.angular.z = math.radians(new_yaw)

    rospy.loginfo('vel_linear  x: %f', new_x)
    rospy.loginfo('vel_linear  y: %f', -new_y)
    rospy.loginfo('vel_linear  z: %f', new_z)
    rospy.loginfo('vel_angular z: %f', new_yaw)
    rospy.loginfo("-------------------------")

    vel_drone_pub.publish(velocity)

    drone_odom = Odometry()
    drone_odom.header.stamp = rospy.Time.now()
    drone_odom.header.frame_id = "odom_nav"
    drone_odom.header.seq = Keyframe
    drone_odom.child_frame_id = "odom"

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, yaw_raw)

    # set the position
    drone_odom.pose.pose = Pose(Vector3(current_x,y_raw,z_raw), Quaternion(*odom_quat))

    tf_br = tf.TransformBroadcaster()
    tf_br.sendTransform(
                  (current_x,y_raw,z_raw), 
                  odom_quat, 
                  drone_odom.header.stamp, 
                  "odom_nav",
                  "odom") #world

    last_time = current_time
    Keyframe += 1

    odom_drone_pub.publish(drone_odom)
    rate.sleep()

def callbackNavHough(data):
  global vel_hough

  vel_hough = data


def callbackRCNN(data):
  global pose_rcnn

  pose_rcnn = data

  ###############################################################################
   
if __name__ == '__main__':
  settings = termios.tcgetattr(sys.stdin)
  rospy.init_node('autonomous_navigation',log_level=rospy.DEBUG)

  # create the important subscribers
  hough_sub = rospy.Subscriber("navigation/nav_position",Twist, callbackNavHough, queue_size = 10)
  rcnn_sub = rospy.Subscriber('rcnn/nav_position', Vector3, callbackRCNN, queue_size=10)

  # create the important publishers
  odom_drone_pub = rospy.Publisher("navigation/nav_odom",Odometry, queue_size = 10)
  cam_pub = rospy.Publisher("bebop/camera_control",Twist, queue_size = 10)
  vel_drone_pub = rospy.Publisher("bebop/cmd_vel",Twist, queue_size = 10)

  # create the publishers to take off and land
  takeoff_pub = rospy.Publisher('bebop/takeoff', Empty, queue_size = 10) # add a publisher for each new topic
  land_pub = rospy.Publisher('bebop/land', Empty, queue_size = 10)    # add a publisher for each new topic
  
  empty_msg = Empty() 

  rate = rospy.Rate(100.0) #-- 100Hz

  print('Program Started')
  print(msg)

  try:
    while(True):
      key = getKey()
      print('key')  # add a print for each key pressed
      print(key)

      if key == '1': # condition created in order to pressed key 1 and generates the take off of the bebop2
        print('key 1 pressed - Takeoff')
        takeoff_pub.publish(empty_msg) # action to publish it
        print(msg)

      elif key == '2': # condition created in order to pressed key 2 and generates the land of the bebop2
        print('key 2 pressed - Landing')
        land_pub.publish(empty_msg) # action to publish it
        print(msg)

      elif key == '3': # condition created in order to pressed key 3 and generates the moviment of camera
        print('key 3 pressed - MoveCamera')
        moveCamera()
        print(msg)

      elif key == '4': # condition created in order to pressed key 6 and generates Auto-Landing of drone
        print('key 4 pressed - Autonomous-Navigation') 
        autoNavigation()
        print(msg)

      elif key == '\x03': # condition created in order to pressed key Ctrl+c and generates output from the program
          print('Quit work')
          break
      else:
        print('Wrong key!')
        print(msg)

  except rospy.ROSInterruptException:
    print('Erro')
