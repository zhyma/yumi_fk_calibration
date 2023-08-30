import sys
sys.path.append('../../')

import numpy as np
import matplotlib.pyplot as plt
import copy
import rospy

import sys, select, os
import tty, termios


from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import tf
from tf.transformations import quaternion_from_matrix, quaternion_matrix

from math import sqrt, sin, cos, pi

from utils.workspace_tf import workspace_tf, pose2transformation, transformation2pose

def estimate_dist(s1, s2):
    dy = sqrt((s1.r+s2.r)**2-(s1.x-s2.x)**2-(s1.z-s2.z)**2)-abs(s1.y-s2.y)
    return dy

def check_spheres(s1, s2):
    '''
    input: two sphere
    output: true:  collision
            false: no collision
    '''
    l2 = (s1.x-s2.x)**2+(s1.y-s2.y)**2+(s1.z-s2.z)**2
    if l2 <= (s1.r+s2.r)**2:
        dist = estimate_dist(s1, s2)
        return True, dist
    
    return False, -1

class sphere():
    def __init__(self, x, y, z, r):
        self.x = x
        self.y = y
        self.z = z
        self.r = r

    def set(self, ht):
        # pos = np.array([[self.x,self.y,self.z,1]]).T
        # pos = np.dot(ht, pos)
        self.x = ht[0,3]
        self.y = ht[1,3]
        self.z = ht[2,3]

class one_hand():
    '''
    the collision model of an individual hand

    '''
    def __init__(self):
        # spheres[0]: palm, spheres[1-3]: green finger, sphere[4-6]: purple finger
        self.spheres = [sphere(0,0,0,0.005) for i in range(7)]
        self.spheres[0].r = 0.05
        ## from yumi_link_7_l or r to palm center or fingers
        #default open
        self.open = True
        self.tdh_open = [np.eye(4) for i in range(7)]
        self.tdh_open[0][2,3] = 0.084

        [self.tdh_open[i].__setitem__((0, 3), 0.03) for i in [1,2,3]]
        self.tdh_open[1][2,3] = 0.126 + 0.005
        self.tdh_open[2][2,3] = 0.126 + 0.052/2
        self.tdh_open[3][2,3] = 0.126 + 0.052 - 0.005

        [self.tdh_open[i].__setitem__((0, 3), -0.03) for i in [4,5,6]]
        self.tdh_open[4][2,3] = 0.126 + 0.005
        self.tdh_open[5][2,3] = 0.126 + 0.052/2
        self.tdh_open[6][2,3] = 0.126 + 0.052 - 0.005
        
        #close
        self.tdh_close = copy.deepcopy(self.tdh_open)
        [self.tdh_close[i].__setitem__((0, 3), 0.006) for i in [1,2,3]]
        [self.tdh_close[i].__setitem__((0, 3), -0.006) for i in [4,5,6]]
        
    # def get_pose(self):
    #     '''
    #     return a 7x3 matrix, which contains the x,y,z of all 7 spheres
    #     '''
    #     pos = np.array([[i.x,i.y,i.z] for i in self.spheres])
    #     return pos

    def set_pose(self, gripper_ht, state = -1):
        '''
        set the pose of link7, calculate collision model pose
        input:
            - gripper_ht: a 4x4 numpy array (homogeneous transformation matrix)
            - state: default is not to change the state. Set to 0 to close and set to 1 to open
        '''
        if state == 0:
            self.open = False
        elif state == 1:
            self.open = True
        # else: do not change the self.open state

        for i in range(7):
            if self.open:
                self.spheres[i].set(np.dot(gripper_ht, self.tdh_open[i]))
            else:
                self.spheres[i].set(np.dot(gripper_ht, self.tdh_close[i]))

class hands():
    def __init__(self):
        self.left = one_hand()
        self.right = one_hand()

    def check_collision(self, left_ht, right_ht, marker_array=None, render_all=False):
        '''
        Only check a pair of grippers' position (left+right) at one time
        input: 
            The homogenuous transformation matrics of the two hands
            marker_array and render_all

        output: a list of collide spheres, length is 0 when there is no collision
        '''
        detected = []
        dist = []
        self.left.set_pose(left_ht)
        self.right.set_pose(right_ht)
        for i in range(len(self.left.spheres)):
            for j in range(len(self.right.spheres)):
                collision, d = check_spheres(self.left.spheres[i],self.right.spheres[j])
                if collision:
                    detected.append([i,j])
                    dist.append(d)

        if (marker_array is not None):
            if render_all:
                self.render_markers(marker_array,collide=detected)
            else:
                if len(detected)>0:
                    # only render collide 
                    self.render_markers(marker_array,collide=detected)

        if len(detected) > 0:
            return detected, max(dist)
        else:
            return detected, -1


    def render_markers(self, marker_array, collide=[]):
        spheres2markers(self.left,  marker_array, [i[0] for i in collide])
        spheres2markers(self.right, marker_array, [i[1] for i in collide])
        return marker_array


def spheres2markers(hand, marker_array, collide=[]):

    start_id = len(marker_array.markers)
    for i in range(7):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()

        marker.type = 2
        marker.id = start_id+i

        marker.scale.x = hand.spheres[i].r*2
        marker.scale.y = hand.spheres[i].r*2
        marker.scale.z = hand.spheres[i].r*2

        if i in collide:
            # collision detected, red
            marker.color.r = 1
            marker.color.g = 0
            marker.color.b = 0
            marker.color.a = 0.5
        else:
            # no collision, green
            marker.color.r = 0
            marker.color.g = 1
            marker.color.b = 0
            marker.color.a = 0.1
        

        marker.pose.position.x = hand.spheres[i].x
        marker.pose.position.y = hand.spheres[i].y
        marker.pose.position.z = hand.spheres[i].z
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1

        marker_array.markers.append(marker)

def dh_test():
    ws = workspace_tf()
    marker_pub = rospy.Publisher("/hands", MarkerArray, queue_size = 10)

    i = 0
    
    while not rospy.is_shutdown():
        ht_l = ws.get_tf("world", "gripper_l_base")
        ht_r = ws.get_tf("world", "gripper_r_base")

        hand_l.set_pose(ht_l)
        hand_r.set_pose(ht_r)

        marker_array = MarkerArray()

        spheres2markers(hand_l, marker_array)
        spheres2markers(hand_r, marker_array)

        marker_pub.publish(marker_array)
        rospy.rostime.wallsleep(1.0)


if __name__ == '__main__':
    rospy.init_node('collision_test', anonymous=True)
    rospy.sleep(1)

    hand_l = hand()
    hand_r = hand()

    # call "roslaunch rope_storage yumi_gazebo_moveit.launch"
    # call "roslaunch rviz_sim.launch"
    # call "python collision_test.py"
    # collision spheres should be appeared on both hands
    dh_test()
    