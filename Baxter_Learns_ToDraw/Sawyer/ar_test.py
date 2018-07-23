#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from lab4_cam.srv import ImageSrv, ImageSrvResponse
import cv2, time, sys
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from numpy.linalg import *
import math
import sys
import copy
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from tf2_msgs.msg import TFMessage

import message_filters
#Define the callback method which is called whenever this node receives a 
#message on its subscribed topic. The received message is passed as the 
#first argument to callback().
class ar_tracking:

    def __init__(self):
        self.total_markers = 2
        self.subscriber = None
        self.markers = None

    def callback(self, message):
        if len(message.markers) == self.total_markers:
            self.markers = message.markers
            #self.subscriber.unregister()
            rospy.signal_shutdown("ar tags found")

    def begin_tracking(self):
        rospy.init_node('listener', anonymous=True)
        #self.subscriber = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.callback)
        sub = message_filters.Subscriber("/ar_pose_marker", AlvarMarkers)
        sub.registerCallback(self.callback)
        rospy.spin()
        return self.markers

class image_rec:

    def __init__(self):
        self.total_markers = 4
        self.subscriber = None
        self.markers = None
        self.image_sub = None
        self.bridge = CvBridge()
        self.save=None

    def begin_tracking(self):
        rospy.init_node('image_listener', anonymous=True)
        self.subscriber = rospy.Subscriber("/io/internal_camera/head_camera/image_raw",Image,self.function,queue_size=1000)
        rospy.spin()
        return self.save


    def function(self,data):
        print "xxxxxxxxxxxxxxxxxxxxxxxxxxx"
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            img2 = img
        except CvBridgeError as e:
            print(e)
        global approx
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(gray,150,255,1)
        cv2.imshow('thresh', thresh)
        dst = cv2.cornerHarris(thresh, 2, 3, 0.04)
        dst = cv2.dilate(dst,None)
        cv2.imshow('dst', img)
        contours,h = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        rect = None
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt,0.02*cv2.arcLength(cnt,True),True)
            if len(approx)==4:
                if cv2.contourArea(approx) > 10000 and cv2.contourArea(approx) < 1000000:
                    self.save = approx
                    cv2.drawContours(img2,[approx],0,(0,255,0),-1)
        print "cccccccccccccccccccccccccc"
        cv2.imshow('img2', img2)
        self.subscriber.unregister()
        rospy.signal_shutdown("ar tags found")
        cv2.waitKey(10000)

def main():
    tracker = ar_tracking()
    print (tracker.begin_tracking())
    print ("hi")
    image = image_rec()
    print (image.begin_tracking())

    ########

    # get image
    # homeography or w/e its called
    # plan drawing
    # do drawing on whiteboard

if __name__ == '__main__':
    main()

