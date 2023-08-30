'''
YuMi's forward kinematics
'''
import sys
sys.path.append('../../')

import sys, copy, pickle, os, time

import numpy as np
from math import pi, sin, cos

import rospy, moveit_commander

from utils.robot.jointspace_ctrl import joint_ctrl

from utils.workspace_tf import workspace_tf, pose2transformation, transformation2pose
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, PoseArray


def yumi_fk(qs, side=0):
    '''
    Solve YuMi's forward kinematics
    Input:
        - a len=7 list, [q1, q2, q7, q3, q4, q5, q6]
        - side == 0 -> left
          side == 1 -> right
          default left
    Output: 
        - a list contins 7 numpy arrays, indicating the pose of link1, link2, link3, link4, ...
          each numpy array is a 4x4 homogeneous matrix
    '''
    ts = []
    ht = [] ## homogenous transformation matrix from joint to joint
    # t = np.eyes(4)
    ## left arm offset
    ht.append(np.array([[-0.57156128, -0.10484,     0.8138343,   0.05355   ],
                        [ 0.61696926, -0.7087945,   0.34199312,  0.0725    ],
                        [ 0.54098672,  0.69758077,  0.46980255,  0.51492   ],
                        [ 0.        ,  0.        ,  0.        ,  1.        ]]))

    ## right arm offset
    ht.append(np.array([[-0.57125902,  0.10708908,  0.81375369,  0.05355   ],
                        [ -0.6197542, -0.70629809,  -0.3421224,  -0.0725   ],
                        [  0.5381151, -0.69976777,  0.46984806,  0.51492   ],
                        [ 0.        ,  0.        ,  0.        ,  1.        ]]))

    ## joint 1
    ht.append(np.array([[ cos(qs[0]), -sin(qs[0]),  0,   0   ],
                        [ sin(qs[0]),  cos(qs[0]),  0,   0   ],
                        [          0,           0,  1,   0   ],
                        [          0,           0,  0,   1.  ]]))

    ## joint 2
    ht.append(np.array([[ cos(qs[1]), -sin(qs[1]),  0,   0.03 ],
                        [          0,           0, -1,   0    ],
                        [ sin(qs[1]),  cos(qs[1]),  0,   0.1  ],
                        [          0,           0,  0,   1.   ]]))

    ## joint 7 as qs[2]
    ht.append(np.array([[ cos(qs[2]), -sin(qs[2]),  0,  -0.03 ],
                        [          0,           0,  1,   0.17283],
                        [-sin(qs[2]), -cos(qs[2]),  0,   0    ],
                        [          0,           0,  0,   1.   ]]))

    ## joint 3
    ht.append(np.array([[-sin(qs[3]), -cos(qs[3]),  0,  -0.04188],
                        [          0,           0, -1,   0    ],
                        [ cos(qs[3]), -sin(qs[3]),  0,   0.07873],
                        [          0,           0,  0,   1.   ]]))

    ## joint 4
    ht.append(np.array([[ cos(qs[4]), -sin(qs[4]),  0,   0.0405],
                        [          0,           0,  1,   0.16461],
                        [-sin(qs[4]), -cos(qs[4]),  0,   0    ],
                        [          0,           0,  0,   1.   ]]))

    ## joint 5
    ht.append(np.array([[ cos(qs[5]), -sin(qs[5]),  0,  -0.027],
                        [          0,           0, -1,   0    ],
                        [ sin(qs[5]),  cos(qs[5]),  0,   0.10039],
                        [          0,           0,  0,   1.   ]]))

    ## joint 6
    ht.append(np.array([[ cos(qs[6]), -sin(qs[6]),  0,   0.027],
                        [          0,           0,  1,   0    ],
                        [-sin(qs[6]), -cos(qs[6]),  0,   0    ],
                        [          0,           0,  0,   1.   ]]))

    if side == 0:
        t = ht[0]
        for i in range(2,9):
            t = np.dot(t,ht[i])
            ts.append(t)

    if side == 1:
        t = ht[1]
        for i in range(2,9):
            t = np.dot(t,ht[i])
            ts.append(t)

    return ts


if __name__=="__main__":
    rospy.init_node('get_yumi_fk', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()
    ws_tf = workspace_tf()

    ctrl_group = moveit_commander.MoveGroupCommander('both_arms')
    j_ctrl = joint_ctrl(ctrl_group)
    ## initialzing the yumi motion planner
    #self.yumi = move_yumi(self.robot, self.scene, self.ctrl_group, self.j_ctrl)

    joints = [-1.1694, -2.3213, 1.1694, -0.3665, 0.2792, 1.2392, -0.3491]+[1.1694, -2.3213, -1.1694, -0.3665, 0.2792, 1.2392, -0.3491]
    # joints = [0, 0, 0, 0, 0, 0, 0]+[0,0,0,0,0,0,0]
    j_ctrl.exec([joints]*5, 0.2)


    pa = PoseArray()
    pa.header.frame_id = "world"
    pa_pub = rospy.Publisher("/arm_pose", PoseArray, queue_size = 10)

    p = Pose()

    while not rospy.is_shutdown():
        ts = yumi_fk(joints[0:7], side=0)
        for i in range(7):
            p = transformation2pose(ts[i])
            pa.poses.append(p)

        ts = yumi_fk(joints[7:], side=1)
        for i in range(7):
            p = transformation2pose(ts[i])
            pa.poses.append(p)


        rospy.sleep(1)
        pa_pub.publish(pa)
        rospy.spin()
