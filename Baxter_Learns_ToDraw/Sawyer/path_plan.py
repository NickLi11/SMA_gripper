#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped, Pose
from baxter_interface import gripper as robot_gripper
import math
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from tf.transformations import quaternion_from_euler, quaternion_multiply
import tf2_ros
from tf2_msgs.msg import TFMessage
from tf2_geometry_msgs import PointStamped
from sensor_msgs.msg import Image

class Pathfinder:

    def __init__(self):
        
        self.markerTransforms = None

        # Grip Settings
        self.gripDist = 51
        self.gripForce = 5

    def main(self):
        #Initialize moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)

        #Start a node
        rospy.init_node('pathfinder')

        #Initialize arm
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        print robot.get_group_names()

    #    left_arm = moveit_commander.MoveGroupCommander('left_arm')
        right_arm = moveit_commander.MoveGroupCommander('right_arm')
    #    left_arm.set_planner_id('RRTConnectkConfigDefault')
    #    left_arm.set_planning_time(10)
        right_arm.set_planner_id('RRTConnectkConfigDefault')
        right_arm.set_planning_time(10)

        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        # #Define start joint states
        # start_joints = right_arm.get_current_joint_values()

        # start_joints[0]=0.25            #base rotate
        # start_joints[1]=-0.2             #first pitch
        # start_joints[2]=0.9           #first revolute
        # start_joints[3]=0.9            #second pitch
        # start_joints[4]=0.0             #second revolute
        # start_joints[5]=0.9             #third pitch
        # start_joints[6]=0.0             #third revolute

        # right_arm.set_joint_value_target(start_joints)

        # right_arm.set_start_state_to_current_state()
        # initial_plan = right_arm.plan()

        # raw_input('Press <Enter> to move the right arm to goal pose: ')
        # right_arm.execute(initial_plan)

        # rospy.sleep(2.0)

        #First goal pose ------------------------------------------------------
        init_pose = PoseStamped()
        init_pose.header.frame_id = "base"
        # #x, y, and z position
        # init_pose.pose.position.x = 0.75
        # init_pose.pose.position.y = -0.5
        # init_pose.pose.position.z = 0.25

        init_pose.pose.position.x = 1.027
        init_pose.pose.position.y = -0.351
        init_pose.pose.position.z = 0.438
        
        # #Orientation as a quaternion
        # init_pose.pose.orientation.x = 0.0
        # init_pose.pose.orientation.y = 0.2588
        # init_pose.pose.orientation.z = 0.0
        # init_pose.pose.orientation.w = 0.9659

        init_pose.pose.orientation.x = -0.175
        init_pose.pose.orientation.y = 0.758
        init_pose.pose.orientation.z = 0.218
        init_pose.pose.orientation.w = 0.590

        right_arm.set_pose_target(init_pose)

        # right_arm.set_start_state_to_current_state()
        # init_plan = right_arm.plan()

        # raw_input('Press <Enter> to move the right arm to goal pose: ')
        # right_arm.execute(init_plan)
        # rospy.sleep(2.0)

        tracker = ar_tracking()
        markers = tracker.begin_tracking()
        # self.markerTransforms = tracker.marker_transform()

        ar_pose = PoseStamped()
        ar_pose.header.frame_id = "base"

        rot = quaternion_from_euler(0, math.pi, 0)

        quat = markers[0].pose.pose.orientation

        result = quaternion_multiply([quat.w, quat.x, quat.y, quat.z], rot)

        ar_pose.pose.orientation.x = result[1]
        ar_pose.pose.orientation.y = result[2]
        ar_pose.pose.orientation.z = result[3]
        ar_pose.pose.orientation.w = result[0]

        pointFromAR = PointStamped()
        pointFromAR.header.frame_id = "ar_marker_4"
        pointFromAR.point.x = 0.1
        pointFromAR.point.y = -0.1
        pointFromAR.point.z = 0.0

        newPoint = tfBuffer.transform(pointFromAR, "base", timeout=rospy.Duration(5.0), new_type = PointStamped)

        print newPoint

        ar_pose.pose.position = newPoint.point

        right_arm.set_pose_target(ar_pose)

        right_arm.set_start_state_to_current_state()
        ar_plan = right_arm.plan()

        raw_input('Press <Enter> to move the right arm to goal pose: ')
        right_arm.execute(ar_plan)
        rospy.sleep(2.0)

        # dots = [np.array([[0.75, -0.25, 0.25]]), 
        #         np.array([[0.9, -0.25, 0.5]]), 
        #         np.array([[0.9, -0.5, 0.5]]), 
        #         np.array([[0.75, -0.5, 0.25]])]

        # dots = [np.array([[ 0.84648066, -0.21843448,  0.01900384]]), 
        #         np.array([[ 0.84314291, -0.21944111,  0.01318214]]), 
        #         np.array([[ 0.84314291, -0.21944111,  0.01318214]])]

        # endPoint = np.array([[0.5, 0.0, 0.25]])

        # end_pose = self.markerTransforms[1].pose.pose

        # dots = [np.array([[end_pose.position.x, end_pose.position.y, end_pose.position.z]])]

        # lastDot = np.array([[init_pose.pose.position.x-0.2, 
        #                      init_pose.pose.position.y, 
        #                      init_pose.pose.position.z+0.1]]
        #                      )
        lastDot = np.array([[ 0.84272569, -0.21956694,  0.01245443]])

        for dot in dots:
            line = self.positionsToWaypoints(lastDot, 
                                             init_pose.pose.orientation, 
                                             dot, 
                                             10)
            # print line
            lastDot = dot

            right_arm.set_start_state_to_current_state()
            rospy.sleep(1.0)

            # (cartPlan, fraction) = right_arm.compute_cartesian_path(line, 0.01, 0.0)
            # rospy.sleep(1.0)
            raw_input('Press <Enter> to execute cartesian path: ')
            # right_arm.execute(cartPlan)
            # rospy.sleep(2.0)

    def findPointsOnLine(self, start, end, steps):
        vector = end - start

        intervals = np.linspace(0.0, 1.0, num=steps+1, endpoint=True)

        distMat = np.array([intervals]).T.dot(vector)

        startMat = np.repeat(start, steps+1, axis=0)

        endMat = startMat + distMat

        return endMat

    def positionsToWaypoints(self, startPoint, startOrientation, endPoint, steps):

        points = self.findPointsOnLine(startPoint, endPoint, steps)

        def pointToWaypoint(point, startOrientation):
            waypoint = Pose()

            waypoint.position.x = point[0]
            waypoint.position.y = point[1]
            waypoint.position.z = point[2]

            waypoint.orientation = startOrientation

            return waypoint

        waypointsMat = np.apply_along_axis(pointToWaypoint, 1, points, startOrientation)

        return waypointsMat.flatten()[1:].tolist()

    def setGripper(self):

        right_gripper = robot_gripper.Gripper('right')

        print('Calibrating gripper')
        right_gripper.calibrate()
        rospy.sleep(2.0)

        print('Opening gripper')
        right_gripper.open()
        rospy.sleep(1.0)
        raw_input('Press <Enter> to close gripper: ')
        right_gripper.command_position(self.gripDist)
        right_gripper.set_holding_force(self.gripForce)
        right_gripper.set_moving_force(self.gripForce)
        # right_gripper.set_dead_band(45)
        rospy.sleep(1.0)
        print('Done!')

        return

class ar_tracking:

    def __init__(self):
        self.total_markers = 3
        self.subscriber = None
        self.markers = None
        self.transforms = {}

    def callback(self, message):
        # print message.markers
        if len(message.markers) == self.total_markers:
            self.markers = message.markers
            self.subscriber.unregister()
            rospy.signal_shutdown("ar tags found")

    def begin_tracking(self):
        # rospy.init_node('ar_track_listener', anonymous=True)
        self.subscriber = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.callback)
        rospy.spin()
        # print self.markers
        return self.markers

    # def test_transform

    # def marker_transform(self):
    #     # rospy.init_node('transform_listener')

    #     tfBuffer = tf2_ros.Buffer()
    #     listener = tf2_ros.TransformListener(tfBuffer)

    #     # tableToBoardTransform = rospy.Publisher('/t2b_transform', geometry_msgs.msg.Transform, queue_size=1)

    #     # rate = rospy.Rate(10.0)
    #     # while not rospy.is_shutdown():
    #     # print self.markers

    #     for marker in self.markers:
    #         print marker.id
    #         try:
    #             trans = tfBuffer.lookup_transform("ar_marker_"+str(marker.id), "base",rospy.Time(), timeout=rospy.Duration(3.0))
    #         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #         #         rate.sleep()
    #             print "transform lookup failed"
    #             continue    

    #         # tableToBoardTransform.publish(trans)
    #         self.transforms[marker.id] = trans

    #     return self.transforms

if __name__ == '__main__':
    sojourner = Pathfinder()
    sojourner.main()
