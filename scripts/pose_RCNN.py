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
        
        current_time = rospy.Time.now()
        last_time = rospy.Time.now()
        vy = 0

        r = rospy.Rate(100.0)
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()

            # compute odometry in a typical way given the velocities of the robot
            dt = (current_time - last_time).to_sec()
            #vy = vy + 

            rospy.logdebug("Altura Filtrada (out): %f", self.msg_nav.z)
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

        list_positivey = []
        list_negativey = []
        list_z = []

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
                medy_ant_p = sum(list_positivey)/len(list_positivey)
                rospy.logdebug("position.pos.y: %f", medy_ant_p)
                self.msg_nav.x = medy_ant_p
            if len(list_negativey) >= 1:
                medy_ant_n = sum(list_negativey)/len(list_negativey)
                rospy.logdebug("position.neg.y: %f", medy_ant_n)
                self.msg_nav.y = medy_ant_n
        rospy.logdebug("--------------------------------")

        #for
        # rospy.logdebug("Tamanho da lista(out): %f", len(list_z))
        # rospy.logdebug("Somatoria lista(out): %f", sum(list_z))
        # rospy.logdebug("Altura Filtrada (out): %f", self.msg_nav.z)
        # rospy.logdebug("--------------------------------")


if __name__=='__main__':
    estimator = Estimator()