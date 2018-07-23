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