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
import geometry_msgs.msg
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from tf2_msgs.msg import TFMessage
import message_filters
from skimage.feature import hog
from sklearn.externals import joblib
from baxter_interface.camera import CameraController
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped
import path_plan

import std_srvs.srv


# Nominal length of a tile side
DRAW_LENGTH = 100 #cm
DRAW_WIDTH = 150
bridge = CvBridge()
approx=0
nx = 1
ny = 1

# Helper function to check computed homography
# This will draw dots in a grid by projecting x,y coordinates
# of tile corners to u,v image coordinates
def check_homography(image, H, nx, ny, length=DRAW_LENGTH):
  # H should be a 3x3 numpy.array
  # nx is the number of tiles in the x direction
  # ny is the number of tiles in the y direction
  # length is the length of one side of a tile
  # image is an image array
  for i in range(nx+1):
    for j in range(ny+1):
      xbar = np.array([[i*length],[j*length],[1]])
      ubar = np.dot(H,xbar).T[0]
      u = np.int(ubar[0]/ubar[2])
      v = np.int(ubar[1]/ubar[2])
      print 'Dot location: ' + str((u,v))
      cv2.circle(image, (u,v), 5, 0, -1)
    cv2.imshow('Check Homography', image)

class ar_tracking:

    def __init__(self):
        self.total_markers = 3
        self.markers = None
        self.sub = None

    def callback(self, message):

        if len(message.markers) >= self.total_markers:
            self.markers = message.markers
            self.sub.unregister()
            print ('shutting down')
            #cam = CameraController('right_hand_camera')
            #cam.close()
            rospy.signal_shutdown("ar tags found")

    def begin_tracking(self):
        #self.image_sub=rospy.Subscriber("io/internal_camera/head_camera/image_raw",Image,self.function,queue_size=1000)
        #self.subscriber1 = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.callback,queue_size=10)
        self.sub = message_filters.Subscriber("/ar_pose_marker", AlvarMarkers)
        self.sub.registerCallback(self.callback)
        rospy.spin()

        return self.markers

class image_tracking:
  def __init__(self):
        self.subscriber = None

        self.sub = None
        self.bridge = CvBridge()
        self.save=None
        self.image=None

  def begin_tracking(self):

        msg = rospy.wait_for_message("/cameras/left_hand_camera/image", Image)
        rospy.sleep(1.0)
        #self.sub = message_filters.Subscriber("/cameras/right_hand_camera/image", Image)
        #self.sub.registerCallback(self.function)
        #rospy.spin()
        self.function(msg)

        return self.save, self.image

  def function(self,data):
      print "xxxxxxxxxxxxxxxxxxxxxxxxxxx"
      try:
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv2.imshow('img', img)
        img2 = img
      except CvBridgeError as e:
        print(e)
      global approx

      gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
      ret, thresh = cv2.threshold(gray, 40, 255, cv2.THRESH_BINARY)
      #thresh = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,2)
      cv2.imshow('thresh', thresh)
      dst = cv2.cornerHarris(thresh, 2, 3, 0.04)
      dst = cv2.dilate(dst,None)
      #cv2.imshow('dst', dst)
      self.image = img

      _, contours, h = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
      rect = None
      max_area = 0
      for cnt in contours:
        approx = cv2.approxPolyDP(cnt,0.02*cv2.arcLength(cnt,True),True)
        if len(approx)==4:
          if cv2.contourArea(approx) > max_area:
          #if cv2.contourArea(approx) > 700 and cv2.contourArea(approx) < 10000:
            self.save = approx
            max_area = cv2.contourArea(approx)
            #cv2.drawContours(img2,[approx],0,(0,255,0),-1)
      print "cccccccccccccccccccccccccc"
      #cv2.imshow('img2', img2)
      #cv2.waitKey(100)
      print ('shutting down pic')
      #rospy.signal_shutdown("BYE")
      #self.sub.unregister()

def ros_to_np_img(ros_img_msg):
  return np.array(bridge.imgmsg_to_cv2(ros_img_msg,'bgr8'))

def reset_cameras():
  try:
    reset_srv = rospy.ServiceProxy('cameras/reset', std_srvs.srv.Empty)
    rospy.wait_for_service('cameras/reset', timeout=10)
    reset_srv()

  except:
      srv_ns = rospy.resolve_name('cameras/reset')
      rospy.logerr("Failed to call reset devices service at %s", srv_ns)
      raise

# Define the total number of clicks we are expecting (4 corners)
TOT_CLICKS = 1

if __name__ == '__main__':
  
  # Waits for the image service to become available
  #rospy.wait_for_service('last_image')
  # Initializes the image processing node
  rospy.init_node('image_processing_node')  
  #last_image_service = rospy.ServiceProxy('last_image', ImageSrv)
  
  print('camera reset')
  reset_cameras()
  rospy.sleep(2.0) 

  try:
    cam = CameraController('head_camera')
    cam.close()
  except:
    print ('head_camera not closed')

  cam = CameraController('left_hand_camera')
  cam.resolution = (1280, 800)
  cam.open()
  rospy.sleep(5.0)    

  print('left_hand_camera opened')

  tracker = image_tracking()

  contours, img = tracker.begin_tracking()
  
  print ('sup')

  print('camera reset')
  reset_cameras()
  rospy.sleep(2.0)

  # try:
  #   cam = CameraController('left_hand_camera')
  #   cam.close()
  # except:
  #   print ('oh no')
  try:
    cam = CameraController('head_camera')
    cam.close()
  except:
    print ('head_camera not closed')
  
  cam = CameraController('right_hand_camera')
  cam.resolution = (1280, 800)
  cam.open()
  rospy.sleep(5.0)

  print('right_hand_camera opened')

  # raw_input("Enter to conitnue:")
  
  ar_tracker = ar_tracking()
  markers = ar_tracker.begin_tracking()
  
  img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
  #cv2.imshow('gray', img_gray)
  #cv2.waitKey()

  contours_copy = contours.co+0.
  py()

  contours = np.vstack(contours).squeeze()


  x,y,w,h = cv2.boundingRect(contours_copy)
  clf = joblib.load("digits_cls.pkl")
  #img = img[y:y+h,x:x+w]

  print (contours)
  c_ordered = [0,0,0,0]
  x = 0
  y = 0
  for c in contours:
    x += c[0]
    y += c[1]
  x /= 4
  y /= 4

  for c in contours:
    if c[0] < x:
      if c[1] < y:
        c_ordered[3] = c
      else:
        c_ordered[2] = c
    else:
      if c[1] < y:
        c_ordered[0] = c
      else:
        c_ordered[1] = c

  pts1 = np.float32([c_ordered[0],c_ordered[1],c_ordered[2],c_ordered[3]])
  print(pts1)
  pts2 = np.float32([[300, 0],[300, 300],[0, 300],[0, 0]])
  M = cv2.getPerspectiveTransform(pts1, pts2)
  cv2.drawContours(img_gray, contours_copy, -1, (255,0,0), 3)
  dst = cv2.warpPerspective(img_gray, M, (300, 300))
  cv2.imshow("ge",img_gray)
  cv2.imshow("warp0",dst)
  dst = dst[15:285,15:285]



  #dst = cv2.GaussianBlur(dst, (5, 5), 0)

  # Threshold the image

  #kernel = np.ones((5,5),np.uint8)    
  ret, dst = cv2.threshold(dst, 64, 255, cv2.THRESH_BINARY)



  img = dst.copy()

  _, ctrs, hier = cv2.findContours(dst.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
  cv2.drawContours(img, ctrs, -1, (255,0,0), 3)

  # Get rectangles contains each contour
  rects = [cv2.boundingRect(ctr) for ctr in ctrs if cv2.contourArea(ctr) > 100 and cv2.contourArea(ctr)<4000]

  # For each rectangular region, calculate HOG features and predict
  # the digit using Linear SVM.


  digits=[]
  positions=[]
  # x, y, w, h
  for rect in rects:
      # Draw the rectangles
      cv2.rectangle(img, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (0, 255, 0), 3) 
      # Make the rectangular region around the digit

      
      #leng = int(rect[3] * 1.6)
      #pt1 = int(rect[1] + rect[3] // 2 - leng // 2)
      #pt2 = int(rect[0] + rect[2] // 2 - leng // 2)
      
      w = rect[2]
      h = rect[3]

      #leng = max(rect[2], rect[3])
      pt1 = rect[1]
      pt2 = rect[0]

      midpoint = [rect[0] + rect[2]/2, rect[1]+rect[3]/2]
      positions.append(midpoint)


      #cv2.circle(img,(rect[0] + rect[2]/2,rect[1] + rect[3]/2), 5, (0,0,255), -1)

      roi = dst[pt1:pt1+rect[3], pt2:pt2+rect[2]]
      #roi = dst[pt1:pt1+leng, pt2:pt2+leng]
      if len(roi) < 1:
          continue
      if len(roi[0]) < 1:
          continue
      # Resize the image
      roi = cv2.resize(roi, (28, 28), interpolation=cv2.INTER_AREA)
      roi = cv2.dilate(roi, (3, 3))

      cv2.imshow('roi', roi)
      print(positions)
      cv2.waitKey()
      # Calculate the HOG features
      roi_hog_fd = hog(roi, orientations=9, pixels_per_cell=(14, 14), cells_per_block=(1, 1), visualise=False)
      nbr = clf.predict(np.array([roi_hog_fd], 'float64'))
      if int(nbr[0]) == 7:
      	digits.append(1)
      elif int(nbr[0]) == 9:
      	digits.append(4)
      else:
      	digits.append(int(nbr[0]))
      print(str(int(nbr[0])))
      #cv2.putText(img, str(int(nbr[0])), (rect[0], rect[1]),cv2.FONT_HERSHEY_DUPLEX, 1, (0, 255, 255), 3)

  #try:
  # rospy.spin()
  #except KeyboardInterrupt:
  # print("shuutting down")
  #def on_mouse_click(event,x,y,flag,param):
  #if(event == cv2.EVENT_LBUTTONUP):
    #point = (x,y)
    #print "Point Captured: " + str(point)
    #points.append(point)
  #while not rospy.is_shutdown():
  np_image = dst
  cv2.imshow("CV Image", img)
  #cv2.waitKey()

  yx = zip(digits,positions)
  yx.sort()
  y_sorted = [y for x,y in yx]


  # y_sorted = [[133, 46], [73, 212], [215, 115], [49, 113], [198, 224]]

  marker0 = None
  marker1 = None
  marker2 = None

  for marker in markers:
    if marker.id == 5:
      marker0 = marker
    elif marker.id == 8:
      marker2 = marker
    else:
      marker1 = marker



  p0 = np.array([marker0.pose.pose.position.x, marker0.pose.pose.position.y, marker0.pose.pose.position.z])
  p1 = np.array([marker1.pose.pose.position.x, marker1.pose.pose.position.y, marker1.pose.pose.position.z])
  p2 = np.array([marker2.pose.pose.position.x, marker2.pose.pose.position.y, marker2.pose.pose.position.z])


  print (p0)
  print (p1)
  print (p2)

  real_positions = []

  for pose in y_sorted:
    pt = p0 + (p1-p0)*(pose[0]/300.00) + (p2-p0)*(300.0-pose[1])/300.0
    real_positions.append(pt)

  print real_positions

  moveit_commander.roscpp_initialize(sys.argv)


  #Initialize both arms
  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()
  #    left_arm = moveit_commander.MoveGroupCommander('left_arm')
  right_arm = moveit_commander.MoveGroupCommander('right_arm')
  #    left_arm.set_planner_id('RRTConnectkConfigDefault')
  #    left_arm.set_planning_time(10)



  right_arm.set_planner_id('RRTConnectkConfigDefault')
  right_arm.set_planning_time(10)

  """
  planner = path_plan.Pathfinder()
  line = planner.positionsToWaypoints(np.array(real_positions[0]).resize((1,3)),right_arm.get_current_pose().pose.orientation,np.array(real_positions[1]).resize((1,3)),10)
  right_arm.set_start_state_to_current_state()
  (cartPlan, fraction) = right_arm.compute_cartesian_path(line, 0.01, 0.0)
  right_arm.execute(plan)
  """

  # waypoints = []


  # waypoints.append(right_arm.get_current_pose().pose)

  offset = -0.07
  # offset =  0.0


  
  goal_2 = PoseStamped()
  goal_2.header.frame_id = "base"

  #x, y, and z position
  goal_2.pose.position.x = real_positions[0][0]
  goal_2.pose.position.y = real_positions[0][1]
  goal_2.pose.position.z = real_positions[0][2]  - offset
  
  #Orientation as a quaternion
  goal_2.pose.orientation.x = 0
  goal_2.pose.orientation.y = -1
  goal_2.pose.orientation.z = 0
  goal_2.pose.orientation.w = 0

  #Set the goal state to the pose you just defined
  right_arm.set_pose_target(goal_2)

  #Set the start state for the right arm
  right_arm.set_start_state_to_current_state()
  plan = right_arm.plan()
  rospy.sleep(1.0)
  right_arm.execute(plan)

  rospy.sleep(2.0)

  goal_2 = PoseStamped()
  goal_2.header.frame_id = "base"

  #x, y, and z position
  goal_2.pose.position.x = real_positions[1][0]
  goal_2.pose.position.y = real_positions[1][1]
  goal_2.pose.position.z = real_positions[1][2] - offset
  
  #Orientation as a quaternion
  goal_2.pose.orientation.x = 0
  goal_2.pose.orientation.y = -1
  goal_2.pose.orientation.z = 0
  goal_2.pose.orientation.w = 0

  #Set the goal state to the pose you just defined
  right_arm.set_pose_target(goal_2)

  #Set the start state for the right arm
  right_arm.set_start_state_to_current_state()
  plan = right_arm.plan()
  rospy.sleep(1.0)
  right_arm.execute(plan)

  rospy.sleep(2.0)

  goal_2 = PoseStamped()
  goal_2.header.frame_id = "base"

  #x, y, and z position
  goal_2.pose.position.x = real_positions[2][0]
  goal_2.pose.position.y = real_positions[2][1]
  goal_2.pose.position.z = real_positions[2][2] - offset
  
  #Orientation as a quaternion
  goal_2.pose.orientation.x = 0
  goal_2.pose.orientation.y = -1
  goal_2.pose.orientation.z = 0
  goal_2.pose.orientation.w = 0

  #Set the goal state to the pose you just defined
  right_arm.set_pose_target(goal_2)

  #Set the start state for the right arm
  right_arm.set_start_state_to_current_state()
  plan = right_arm.plan()
  rospy.sleep(1.0)
  right_arm.execute(plan)

  rospy.sleep(2.0)

  goal_2 = PoseStamped()
  goal_2.header.frame_id = "base"

  #x, y, and z position
  goal_2.pose.position.x = real_positions[3][0]
  goal_2.pose.position.y = real_positions[3][1]
  goal_2.pose.position.z = real_positions[3][2] - offset
  
  #Orientation as a quaternion
  goal_2.pose.orientation.x = 0
  goal_2.pose.orientation.y = -1
  goal_2.pose.orientation.z = 0
  goal_2.pose.orientation.w = 0

  #Set the goal state to the pose you just defined
  right_arm.set_pose_target(goal_2)

  #Set the start state for the right arm
  right_arm.set_start_state_to_current_state()
  plan = right_arm.plan()
  rospy.sleep(1.0)
  right_arm.execute(plan)

  rospy.sleep(2.0)

  goal_2 = PoseStamped()
  goal_2.header.frame_id = "base"

  #x, y, and z position
  goal_2.pose.position.x = real_positions[4][0]
  goal_2.pose.position.y = real_positions[4][1]
  goal_2.pose.position.z = real_positions[4][2] - offset
  
  #Orientation as a quaternion
  goal_2.pose.orientation.x = 0
  goal_2.pose.orientation.y = -1
  goal_2.pose.orientation.z = 0
  goal_2.pose.orientation.w = 0

  #Set the goal state to the pose you just defined
  right_arm.set_pose_target(goal_2)

  #Set the start state for the right arm
  right_arm.set_start_state_to_current_state()
  plan = right_arm.plan()
  rospy.sleep(1.0)
  right_arm.execute(plan)

  rospy.sleep(2.0)

  goal_2 = PoseStamped()
  goal_2.header.frame_id = "base"

  #x, y, and z position
  goal_2.pose.position.x = real_positions[0][0]
  goal_2.pose.position.y = real_positions[0][1]
  goal_2.pose.position.z = real_positions[0][2] - offset
  
  #Orientation as a quaternion
  goal_2.pose.orientation.x = 0
  goal_2.pose.orientation.y = -1
  goal_2.pose.orientation.z = 0
  goal_2.pose.orientation.w = 0

  #Set the goal state to the pose you just defined
  right_arm.set_pose_target(goal_2)

  #Set the start state for the right arm
  right_arm.set_start_state_to_current_state()
  plan = right_arm.plan()
  rospy.sleep(1.0)
  right_arm.execute(plan)

  rospy.sleep(2.0)

 