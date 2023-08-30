#!/usr/bin/env python3

from math import pi, sin, cos, sqrt, atan2
import numpy as np
import sys
import copy

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from transforms3d import euler

from tf.transformations import quaternion_from_matrix, quaternion_matrix


class path_generator():

    def __init__(self):
        self.left_ee_pub = rospy.Publisher('yumi_left', Path, queue_size=1, latch=True)
        self.right_ee_pub = rospy.Publisher('yumi_right', Path, queue_size=1, latch=True)

    def generate_circle(self, t_rod, r, side = 0, ee_fixed = False):
        ## default side=0 is left hand
        ## curve on x-z plane
        ## from 2d plane curve to world frame path: [xr, adv, yr]
        path = []
        n_samples = 12

        ## T^{ref}_{obj}: from ref to obj
        ## t_ft2gb: Finger Tip with respect to the Gripper Base
        ## t_gb2ft is the inverse matrix (gb with repesct to ft)
        ## 0.125: from the finger tip to the gripper base
        ## 0.100: from the grasping point to the gripper base
        t_ft2gb = np.array([[1, 0, 0, 0],\
                            [0, 1, 0, 0],\
                            [0, 0, 1, 0.10],\
                            [0, 0, 0, 1]])
        t_gb2ft = np.linalg.inv(t_ft2gb)

        a = 0

        for i in range(n_samples+1):
        # for i in range(3):
            
            t = (2+0)*pi/n_samples * i

            x = r*cos(t+pi/2)
            z = r*sin(t+pi/2)

            t_curve2d = np.array([[1, 0, 0, x],\
                                  [0, 1, 0, 0],\
                                  [0, 0, 1, -z],\
                                  [0, 0, 0, 1]])

            ## project the curve on the plane that perpendicular to the rod
            t_ft2world = np.dot(t_rod, t_curve2d)

            ## tested for the left arm
            st = sin(t+pi)
            ct = cos(t+pi)

            if side == 1:
                ## right arm
                if ee_fixed == False:
                    ## placeholder, haven't test which direction this is yet.
                    t_orientation = np.array([[ ct, st,  0, 0],\
                                              [  0,  0, -1, 0],\
                                              [-st, ct,  0, 0],\
                                              [  0,  0,  0, 1]])
                else:
                    ## placeholder, haven't test which direction this is yet.
                    t_orientation = np.array([[  1,  0,  0, 0],\
                                              [  0,  0,  1, 0],\
                                              [  0, -1,  0, 0],\
                                              [  0,  0,  0, 1]])
            else:
                # default: left arm
                if ee_fixed == False:
                    st = sin(t+pi)
                    ct = cos(t+pi)
                    t_orientation = np.array([[ ct, st,  0, 0],\
                                              [  0,  0, -1, 0],\
                                              [-st, ct,  0, 0],\
                                              [  0,  0,  0, 1]])
                else:
                    ## placeholder, haven't test which direction this is yet.
                    t_orientation = np.array([[  1,  0,  0, 0],\
                                              [  0,  0, -1, 0],\
                                              [  0,  1,  0, 0],\
                                              [  0,  0,  0, 1]])

            t_ft2world[:3,:3] = np.dot(t_rod, t_orientation)[:3,:3]

            ## reference frame:world. t_world2gb = t_world2ft*inv(t_gb2ft)
            t_gb2world = np.dot(t_ft2world,t_gb2ft)
            o = t_gb2world[:3,3]
            q = quaternion_from_matrix(t_gb2world)

            pose = Pose()
            pose.position.x = o[0]
            pose.position.y = o[1]
            pose.position.z = o[2]

            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]

            # theta = (220.0-360.0/n_samples*i)*pi/180.0
            # new_pose = step_back(pose, theta)

            path.append(pose)
        
        return path

    def generate_line(self, start, stop, n_samples = 10):
        path = []

        [x0, y0, z0] = [start.position.x, start.position.y, start.position.z]
        [x1, y1, z1] = [ stop.position.x,  stop.position.y,  stop.position.z]


        for i in range(n_samples):
            pose = copy.deepcopy(stop)
            pose.position.x = x0+(x1-x0)*i/(n_samples-1)
            pose.position.y = y0+(y1-y0)*i/(n_samples-1)
            pose.position.z = z0+(z1-z0)*i/(n_samples-1)
            path.append(pose)

        return path

    def publish_waypoints(self, path, side = 0):
        """
        Publish the ROS message containing the waypoints
        """

        msg = Path()
        msg.header.frame_id = "world"
        msg.header.stamp = rospy.Time.now()

        for wp in path:
            pose = PoseStamped()
            pose.pose.position.x = wp.position.x
            if side == 0:
                pose.pose.position.y = wp.position.y-0.12
            else:
                pose.pose.position.y = wp.position.y+0.12
            pose.pose.position.z = wp.position.z

            pose.pose.orientation.x = wp.orientation.x
            pose.pose.orientation.y = wp.orientation.y
            pose.pose.orientation.z = wp.orientation.z
            pose.pose.orientation.w = wp.orientation.w
            msg.poses.append(pose)

        if side == 0:
            self.left_ee_pub.publish(msg)
        # rospy.loginfo("Published {} waypoints.".format(len(msg.poses)))
        else:
            self.right_ee_pub.publish(msg)
        return 
