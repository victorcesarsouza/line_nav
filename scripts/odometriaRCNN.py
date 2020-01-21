#!/usr/bin/env python
## Author: Alan Tavares
## Date: August, 12, 2019
# Purpose: Ros node to detect objects using tensorflow
import os
import sys
import cv2
import numpy as np
# try:
#     import tensorflow as tsf
# except ImportError:
#     print("unable to import TensorFlow. Is it installed?")
#     print("  sudo apt install python-pip")
#     print("  sudo pip install tensorflow")
#     sys.exit(1)

# ROS related imports
import rospy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry, Path
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

# Estimator
class Estimator(object):

    def __init__(self):
        super(Estimator, self).__init__()
        rospy.init_node('estimator_node', anonymous=True, log_level=rospy.DEBUG)

        self.msg_nav = Odometry()
        
        self.msg_nav.header.frame_id = "odom_rcnn"
        # self.msg_nav.pose.pose.position.x = 0
        # self.msg_nav.pose.pose.position.y = 0
        # self.msg_nav.pose.pose.position.z = 0

        # Publishers
        self.rcnn_pub = rospy.Publisher('rcnn/nav_position', Odometry, queue_size=100)

        # Subscribers
        self.object_sub = rospy.Subscriber("rcnn/objects", Detection2DArray, self.objCallback, queue_size=100)

        current_time = rospy.Time.now()
        last_time = rospy.Time.now()

        r = rospy.Rate(100.0)
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()

            # compute odometry in a typical way given the velocities of the robot
            dt = (current_time - last_time).to_sec()
            self.msg_nav.header.stamp = current_time
            # self.msg_nav.x = 0
            # self.msg_nav.y = 0
            rospy.logdebug("Altura Filtrada (out): %f", self.msg_nav.pose.pose.position.z)
            rospy.logdebug("--------------------------------")

            self.rcnn_pub.publish(self.msg_nav)

            r.sleep()

        try: 
            rospy.spin()
        except rospy.ROSInterruptException:
            print("Shutting down")

    def objCallback(self, data):
        # recive data
        objArray = Detection2DArray()
        objArray = data
        obj = objArray.detections

        # list_positivex = []
        # list_negativex = []
        list_positivey = []
        list_negativey = []
        list_y = []
        list_z = []

        # Object search
        if len(obj) >= 1:
            for i in range(len(obj)):
                #pointx = objArray.detections[i].results[0].pose.pose.position.x
                pointy = objArray.detections[i].results[0].pose.pose.position.y
                list_z.append(objArray.detections[i].results[0].pose.pose.position.z)
                #rospy.logdebug("position.y [%d]: %f", i, objArray.detections[i].results[0].pose.pose.position.y)
                #rospy.logdebug("position.z [%d]: %f", i, objArray.detections[i].results[0].pose.pose.position.z)

                # # list x
                # if pointx > 0:
                #     list_positivex.append(pointx)
                # else:
                #     list_negativex.append(pointx)
                # list y
                if pointy > 0:
                    list_positivey.append(pointy)
                else:
                    list_negativey.append(pointy)

            medz_ant = sum(list_z)/len(list_z)
            self.msg_nav.pose.pose.position.z = medz_ant

            #medx_ant_p = sum(list_positivex)/len(list_positivex)
            #medx_ant_n = sum(list_negativex)/len(list_negativex)
            if len(list_positivey) >= 1:
                medy_ant_p = sum(list_positivey)/len(list_positivey)
                rospy.logdebug("position.pos.y: %f", medy_ant_p)
            if len(list_negativey) >= 1:
                medy_ant_n = sum(list_negativey)/len(list_negativey)
                rospy.logdebug("position.neg.y: %f", medy_ant_n)



        rospy.logdebug("--------------------------------")

        #for
        

        # rospy.logdebug("Tamanho da lista(out): %f", len(list_z))
        # rospy.logdebug("Somatoria lista(out): %f", sum(list_z))
        # rospy.logdebug("Altura Filtrada (out): %f", self.msg_nav.pose.pose.position.z)
        # rospy.logdebug("--------------------------------")


if __name__=='__main__':
    estimator = Estimator()