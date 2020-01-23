#!/usr/bin/env python
## Author: Alan Tavares
## Date: August, 12, 2019
# Purpose: Ros node to detect objects using tensorflow
import os
import sys
import cv2
import numpy as np

# ROS related imports
import rospy
from geometry_msgs.msg import Twist, Vector3
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

# Estimator
class Estimator(object):

    def __init__(self):
        super(Estimator, self).__init__()
        rospy.init_node('estimator_node', log_level=rospy.DEBUG)

        self.VERBOSE = True
        self.msg_nav = Vector3()
        
        #############################################################################
        # Publishers
        self.rcnn_pub = rospy.Publisher('rcnn/nav_position', Vector3, queue_size=1)

        # Subscribers
        self.object_sub = rospy.Subscriber("rcnn/objects", Detection2DArray, self.objCallback, queue_size=1)

        #############################################################################
        
        # current_time = rospy.Time.now()
        # last_time = rospy.Time.now()
        # current_x = 0
        # last_x = 0
        # Vref = 0.02
        # Vcx = 0

        # r = rospy.Rate(100.0)
        # while not rospy.is_shutdown():
        #     current_time = rospy.Time.now()
        #     current_x = self.msg_nav.y

        #     # compute dt and Vc, given the velocity of the drone
        #     dt = (current_time - last_time).to_sec()
        #     Vcy = (current_x - last_x)/dt
        #     Vout_y = (Vref - Vcy)

        # Yreal = (Ysp*Zreal)/k
        # k = (Ysp*Zreal)/Yreal

        #     rospy.logdebug("Vout_x (out): %f", Vout_y)
        #     rospy.logdebug("X Filtrada (out): %f", self.msg_nav.y)
        #     rospy.logdebug("Z Filtred (out): %f", self.msg_nav.z)
        #     rospy.logdebug("--------------------------------")

        #     self.rcnn_pub.publish(self.msg_nav)

        #     last_time = current_time
        #     last_x = current_x
        #     r.sleep()

        try: 
            rospy.spin()
        except rospy.ROSInterruptException:
            print("Shutting down")

    def objCallback(self, data):
        # recive data
        objArray = Detection2DArray()
        objArray = data
        obj = objArray.detections

        list_positivey = []
        list_negativey = []
        list_z = []
        positivey = False
        negativey = False

        # Object search
        if len(obj) >= 1:
            for i in range(len(obj)):
                pointy = obj[i].results[0].pose.pose.position.y
                list_z.append(obj[i].results[0].pose.pose.position.z)
                #rospy.logdebug("position.y [%d]: %f", i, objArray.detections[i].results[0].pose.pose.position.y)
                #rospy.logdebug("position.z [%d]: %f", i, objArray.detections[i].results[0].pose.pose.position.z)
                # list y
                if pointy > 0:
                    list_positivey.append(pointy)
                else:
                    list_negativey.append(pointy)

            medz_ant = sum(list_z)/len(list_z)
            self.msg_nav.z = medz_ant

            if len(list_positivey) >= 1:
                # mean of list x+
                medy_ant_p = sum(list_positivey)/len(list_positivey)
                # rospy.logdebug("position.pos.y: %f", medy_ant_p)
                #self.msg_nav.x = medy_ant_p
                positivey = True

            if len(list_negativey) >= 1:
                # mean of list x-
                medy_ant_n = sum(list_negativey)/len(list_negativey)
                #rospy.logdebug("position.neg.y: %f", medy_ant_n)
                # self.msg_nav.y = medy_ant_n
                negativey = True

        if negativey == True:
            self.msg_nav.y = medy_ant_n
            # rospy.logdebug("negativey(T) y: %f", self.msg_nav.y)
        elif negativey == False and positivey == True:
            self.msg_nav.y = medy_ant_p
            # rospy.logdebug("position(F) and negativey(T) y: %f", self.msg_nav.y)
        else:
            self.msg_nav.y = 0
        #     rospy.logdebug("No position y: %f", self.msg_nav.y)
        # rospy.logdebug("--------------------------------")

        self.rcnn_pub.publish(self.msg_nav)
        # rospy.logdebug("Tamanho da lista(out): %f", len(list_z))
        # rospy.logdebug("Somatoria lista(out): %f", sum(list_z))
        # rospy.logdebug("Altura Filtrada (out): %f", self.msg_nav.z)
        # rospy.logdebug("--------------------------------")


if __name__=='__main__':
    estimator = Estimator()