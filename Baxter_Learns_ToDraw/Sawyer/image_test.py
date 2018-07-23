#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("io/internal_camera/head_camera/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      img = self.bridge.imgmsg_to_cv2(data, "bgr8")
      img2 = img
    except CvBridgeError as e:
      print(e)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret,thresh = cv2.threshold(gray,50,255,1)

    cv2.imshow('thresh', thresh)

    dst = cv2.cornerHarris(thresh, 2, 3, 0.04)
    #result is dilated for marking the corners, not important
    dst = cv2.dilate(dst,None)

    # Threshold for an optimal value, it may vary depending on the image.
    #img[dst>0.25*dst.max()]=[0,0,255]

    cv2.imshow('dst', img)
    contours,h = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


    rect = None

    for cnt in contours:
        approx = cv2.approxPolyDP(cnt,0.02*cv2.arcLength(cnt,True),True)
        if len(approx)==4:
            if cv2.contourArea(approx) > 10000 and cv2.contourArea(approx) < 100000:
              print approx
              rect = approx
              cv2.drawContours(img2,[approx],0,(0,255,0),-1)
              #cv2.drawContours(img2,[cnt],0,(0,0,255),-1)
        """
        elif len(approx) > 15:
            if cv2.contourArea(approx) > 100 and cv2.contourArea(approx) < 10000:
              cv2.drawContours(img2,[cnt],0,(0,0,255),-1)
        """

    cv2.imshow('img2', img2)
    
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
