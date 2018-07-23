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
from skimage.feature import hog
from sklearn.externals import joblib


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
        self.total_markers = 1
        self.subscriber1 = None
        self.subscriber2 = None
        self.markers = None
        self.image_sub = None
        self.bridge = CvBridge()
        self.save=None
        self.image=None
    def function(self,data):
      print "xxxxxxxxxxxxxxxxxxxxxxxxxxx"
      try:
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        img2 = img
      except CvBridgeError as e:
        print(e)
      global approx
      gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
      thresh = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_MEAN_C,\
            cv2.THRESH_BINARY,11,2)
      cv2.imshow('thresh', thresh)
      dst = cv2.cornerHarris(thresh, 2, 3, 0.04)
      dst = cv2.dilate(dst,None)
      cv2.imshow('dst', img)
      self.image = img

      contours,h = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
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
      cv2.imshow('img2', img2)
      cv2.waitKey(100)
      rospy.signal_shutdown("BYE")

      # cv2.destroyAllWindows()
    def callback(self, message):

        if len(message.markers) >= self.total_markers:
            self.markers = message.markers
            #self.subscriber1.unregister()
            #rospy.signal_shutdown("ar tags found")

    def begin_tracking(self):
        #self.image_sub=rospy.Subscriber("io/internal_camera/head_camera/image_raw",Image,self.function,queue_size=1000)
        #self.subscriber1 = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.callback,queue_size=10)
        sub1 = message_filters.Subscriber("/ar_pose_marker", AlvarMarkers)
        sub1.registerCallback(self.callback)

        sub2 = message_filters.Subscriber("/cameras/left_hand_camera/image", Image)
        sub2.registerCallback(self.function)

        rospy.spin()
        return self.markers,self.save, self.image
def ros_to_np_img(ros_img_msg):
  return np.array(bridge.imgmsg_to_cv2(ros_img_msg,'bgr8'))

def num_to_hom(num):
  return (p2-p1).*(num[i][0]+150)/300+(450-num[i][1])/300.*(p3-p1)+p1

# Define the total number of clicks we are expecting (4 corners)
TOT_CLICKS = 1

if __name__ == '__main__':
  
  # Waits for the image service to become available
  #rospy.wait_for_service('last_image')
  # Initializes the image processing node
  rospy.init_node('image_processing_node')  
  #last_image_service = rospy.ServiceProxy('last_image', ImageSrv)
  tracker = ar_tracking()
  markers, contours, img = tracker.begin_tracking()
  #cv2.drawContours(img,[contours],0,(0,255,0),-1)
  #p1=markers[0].PoseStamped.pose.position
  #p2=markers[1].PoseStamped.pose.position
  #p3=markers[2].PoseStamped.pose.position  
  p1=np.array([0,0,0])
  p2=np.array([100,0,0])
  p3=np.array([0,100,0])
  img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
  contours_copy = contours.copy()
  contours = np.vstack(contours).squeeze()
  x,y,w,h = cv2.boundingRect(contours_copy)
  clf = joblib.load("digits_cls.pkl")
  #img = img[y:y+h,x:x+w
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
  pts2 = np.float32([[300, 0],[300, 300],[0, 300],[0, 0]])


  M = cv2.getPerspectiveTransform(pts1, pts2)
  dst = cv2.warpPerspective(img_gray, M, (300, 300))


  dst = dst[15:285,15:285]


  dst = cv2.GaussianBlur(dst, (5, 5), 0)

  # Threshold the image

  #kernel = np.ones((5,5),np.uint8)    
  ret, dst = cv2.threshold(dst, 115, 255, cv2.THRESH_BINARY)
  img = dst.copy()  
  cv2.imshow('warp', dst)
  cv2.waitKey()

  ctrs, hier = cv2.findContours(dst.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
  #cv2.drawContours(img, ctrs, -1, (255,0,0), 3)

  # Get rectangles contains each contour
  rects = [cv2.boundingRect(ctr) for ctr in ctrs if cv2.contourArea(ctr) > 350 and cv2.contourArea(ctr)<4000]
  count=0
  sumx=0
  # For each rectangular region, calculate HOG features and predict
  # the digit using Linear SVM.
  for rect in rects:
      # Draw the rectangles
      cv2.rectangle(img, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (0, 255, 0), 3) 
      # Make the rectangular region around the digit

      """
      leng = int(rect[3] * 1.6)
      pt1 = int(rect[1] + rect[3] // 2 - leng // 2)
      pt2 = int(rect[0] + rect[2] // 2 - leng // 2)
      """
      leng = max(rect[2], rect[3])
      pt1 = rect[1]
      pt2 = rect[0]


      #cv2.circle(img,(rect[0] + rect[2]/2,rect[1] + rect[3]/2), 5, (0,0,255), -1)

      roi = dst[pt1:pt1+rect[3], pt2:pt2+rect[2]]
      if len(roi) < 1:
          continue
      if len(roi[0]) < 1:
          continue
      # Resize the image
      roi = cv2.resize(roi, (28, 28), interpolation=cv2.INTER_AREA)
      roi = cv2.dilate(roi, (3, 3))

      cv2.imshow('roi', roi)
      cv2.waitKey()

      # Calculate the HOG features
      roi_hog_fd = hog(roi, orientations=9, pixels_per_cell=(14, 14), cells_per_block=(1, 1), visualise=False)
      count=count+1
      nbr = clf.predict(np.array([roi_hog_fd], 'float64'))
      sumx=sumx+nbr
      num[nbr]=(rect[0]+rect[2]/2,rect[1]+rect[3]/2)
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
  #while not rospy.is_shutdown()
  digit[1]=np.array([0,0,0,10,0,20])
  digit[1]=digit[1].reshape(3,2)
  digit[2]=np.array([0,0,10,0,10,10,0,10,0,20,10,20])
  digit[2]=digit[2].reshape(6,2)
  digit[3]=np.array([0,0,10,0,10,10,0,10,10,10,10,20,0,20])
  digit[3]=digit[3].reshape(7,2)
  digit[4]=np.array([0,0,0,10,10,10,10,0,10,10,10,20])
  digit[4]=digit[4].reshape(6,2)
  digit[5]=np.array([10,0,0,0,0,10,10,10,10,20,0,20])
  digit[5]=digit[5].reshape(6,2)
  digit[6]=np.array([10,0,0,0,0,10,0,20,10,20,10,10,0,10])
  digit[6]=digit[6].reshape(7,2)
  digit[7]=np.array([0,0,10,0,10,10,10,20])
  digit[7]=digit[7].reshape(4,2)
  digit[8]=np.array([0,0,10,0,10,10,0,10,0,0,0,10,0,20,10,20,10,10])
  digit[8]=digit[8].reshape(9,2)
  digit[9]=np.array([0,20,10,20,10,10,10,0,0,0,0,10,10,10])
  digit[9]=digit[9].reshape(7,2)

  for i in range(1,count):
    hom[i]=(p2-p1).*num[i][0]/300+(300-num[i][1])/300.*(p3-p1)+p1
  
  for i in range(1,count+1) :
    moveit_commander.roscpp_initialize(sys.argv)
    #Start a node
    rospy.init_node('moveit_node')

    #Initialize both arms
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
#    left_arm = moveit_commander.MoveGroupCommander('left_arm')
    right_arm = moveit_commander.MoveGroupCommander('right_arm')
#    left_arm.set_planner_id('RRTConnectkConfigDefault')
#    left_arm.set_planning_time(10)
    right_arm.set_planner_id('RRTConnectkConfigDefault')
    right_arm.set_planning_time(10)
    
    #First goal pose ------------------------------------------------------
    #goal_1 = PoseStamped()
    #goal_1.header.frame_id = "base"

    #x, y, and z position
    #goal_1.pose.position.x = 1.08
    #goal_1.pose.position.y = 0.60
    #goal_1.pose.position.z = 0.83
    
    #Orientation as a quaternion
    #goal_1.pose.orientation.x = 0.0
    #goal_1.pose.orientation.y = -1.0
    #goal_1.pose.orientation.z = 0.0
    #goal_1.pose.orientation.w = 0.0

    #Set the goal state to the pose you just defined
    #right_arm.set_pose_target(goal_1)

    #Set the start state for the right arm
    #right_arm.set_start_state_to_current_state()

    #Plan a path
    #right_plan = right_arm.plan()

    #Execute the plan
    #raw_input('Press <Enter> to move the right arm to goal pose 1 (path constraints are never enforced during this motion): ')
    #right_arm.execute(right_plan)

    #Second goal pose -----------------------------------------------------
    #rospy.sleep(2.0)
    goal[i] = PoseStamped()
    goal[i].header.frame_id = "base"

    #x, y, and z position
    goal[i].pose.position.x = hom[i][0]
    goal[i].pose.position.y = hom[i][1]
    goal[i].pose.position.z = hom[i][2]

    #Orientation as a quaternion
    goal[i].pose.orientation.x = 0.0
    goal[i].pose.orientation.y = -1.0
    goal[i].pose.orientation.z = 0.0
    goal[i].pose.orientation.w = 0.0

    #Set the goal state to the pose you just defined
    right_arm.set_pose_target(goal[i])

    #Set the start state for the right arm
    right_arm.set_start_state_to_current_state()

    # #Create a path constraint for the arm
    # #UNCOMMENT TO ENABLE ORIENTATION CONSTRAINTS
    """
    orien_const = OrientationConstraint()
    orien_const.link_name = "right_gripper";
    orien_const.header.frame_id = "base";
    orien_const.orientation.y = -1.0
    orien_const.absolute_x_axis_tolerance = 0.1;
    orien_const.absolute_y_axis_tolerance = 0.1;
    orien_const.absolute_z_axis_tolerance = 0.1;
    orien_const.weight = 1.0;
    consts = Constraints()
    consts.orientation_constraints = [orien_const]
    right_arm.set_path_constraints(consts)
    """
    #Plan a path
    right_plan = right_arm.plan()
    raw_input('Press <Enter> to the next move: ')
    #Execute the plan
    right_arm.execute(right_plan)
  raw_input('Press <Enter> to get the sum of digits: ')
  sum_draw[0]=num_to_hom(digit[(sumx/10)])
  sum_draw[1]=num_to_hom(digit[(sumx%10)])
  for i in range(0,2) :
    moveit_commander.roscpp_initialize(sys.argv)
    #Start a node
    rospy.init_node('moveit_node')

    #Initialize both arms
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
#    left_arm = moveit_commander.MoveGroupCommander('left_arm')
    right_arm = moveit_commander.MoveGroupCommander('right_arm')
#    left_arm.set_planner_id('RRTConnectkConfigDefault')
#    left_arm.set_planning_time(10)
    right_arm.set_planner_id('RRTConnectkConfigDefault')
    right_arm.set_planning_time(10)
    
    #First goal pose ------------------------------------------------------
    #goal_1 = PoseStamped()
    #goal_1.header.frame_id = "base"

    #x, y, and z position
    #goal_1.pose.position.x = 1.08
    #goal_1.pose.position.y = 0.60
    #goal_1.pose.position.z = 0.83
    
    #Orientation as a quaternion
    #goal_1.pose.orientation.x = 0.0
    #goal_1.pose.orientation.y = -1.0
    #goal_1.pose.orientation.z = 0.0
    #goal_1.pose.orientation.w = 0.0

    #Set the goal state to the pose you just defined
    #right_arm.set_pose_target(goal_1)

    #Set the start state for the right arm
    #right_arm.set_start_state_to_current_state()

    #Plan a path
    #right_plan = right_arm.plan()

    #Execute the plan
    #raw_input('Press <Enter> to move the right arm to goal pose 1 (path constraints are never enforced during this motion): ')
    #right_arm.execute(right_plan)

    #Second goal pose -----------------------------------------------------
    #rospy.sleep(2.0)
    goal[i] = PoseStamped()
    goal[i].header.frame_id = "base"

    #x, y, and z position
    goal[i].pose.position.x = sum_draw[i][0]
    goal[i].pose.position.y = sum_draw[i][1]
    goal[i].pose.position.z = sum_draw[i][2]

    #Orientation as a quaternion
    goal[i].pose.orientation.x = 0.0
    goal[i].pose.orientation.y = -1.0
    goal[i].pose.orientation.z = 0.0
    goal[i].pose.orientation.w = 0.0

    #Set the goal state to the pose you just defined
    right_arm.set_pose_target(goal[i])

    #Set the start state for the right arm
    right_arm.set_start_state_to_current_state()

    # #Create a path constraint for the arm
    # #UNCOMMENT TO ENABLE ORIENTATION CONSTRAINTS
    """
    orien_const = OrientationConstraint()
    orien_const.link_name = "right_gripper";
    orien_const.header.frame_id = "base";
    orien_const.orientation.y = -1.0
    orien_const.absolute_x_axis_tolerance = 0.1;
    orien_const.absolute_y_axis_tolerance = 0.1;
    orien_const.absolute_z_axis_tolerance = 0.1;
    orien_const.weight = 1.0;
    consts = Constraints()
    consts.orientation_constraints = [orien_const]
    right_arm.set_path_constraints(consts)
    """
    #Plan a path
    right_plan = right_arm.plan()

    #Execute the plan
    right_arm.execute(right_plan)
