## run `roslaunch rs2pcl demo.launch` first

'''
Test insert new key pose if collision is detected (simulation)
'''

import sys, copy, pickle, os, time, rospy, moveit_commander, cv2

import numpy as np
from math import pi

import time

from utils.workspace_tf          import workspace_tf, pose2transformation, transformation2pose

from utils.robot.workspace_ctrl  import move_yumi
from utils.robot.jointspace_ctrl import joint_ctrl
from utils.robot.path_generator  import path_generator
from utils.robot.gripper_ctrl    import gripper_ctrl
from utils.robot.interpolation   import interpolation
from utils.robot.visualization   import marker
from utils.robot.planning_tools  import replanner

from utils.robot.collision_test  import hands
from utils.robot.yumi_fk         import yumi_fk
from utils.robot.utility_tools   import utility_tools
from utils.robot.utility_tools   import tf_with_offset, pose_with_offset

# from utils.vision.rgb_camera     import image_converter
# from utils.vision.rope_detect    import rope_detect

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion

from find_intersection import path_sync


class robot_wrapping():
    def __init__(self):

        self.gripper = gripper_ctrl()
        ##-------------------##
        ## initializing the moveit 
        moveit_commander.roscpp_initialize(sys.argv)
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()
        self.ws_tf = workspace_tf()

        self.ctrl_group = moveit_commander.MoveGroupCommander('both_arms')
        self.j_ctrl = joint_ctrl(self.ctrl_group)
        ## initialzing the yumi motion planner
        self.yumi = move_yumi(self.robot, self.scene, self.ctrl_group, self.j_ctrl)
        self.pg = path_generator()

        self.hands = hands()

        self.dt = 2

        ## left initial pose
        self.left_default = np.array([[ 0, 1,  0, 0],\
                                      [ 0, 0, -1, 0],\
                                      [-1, 0,  0, 0],\
                                      [ 0, 0,  0, 1]]).astype('float64')
        self.j6l_default = 0

        ## right inital pose
        self.right_default = np.array([[  1,  0,  0, 0],\
                                       [  0,  0,  1, 0],\
                                       [  0, -1,  0, 0],\
                                       [  0,  0,  0, 1]]).astype('float64')
        self.j6r_default = -1

        self.replanner = replanner(self.yumi, self.hands, self.dt)
        self.ut = utility_tools(self.yumi, self.j_ctrl, self.gripper)

    def init_pose(self):
        ...

    def step(self, pts_left, pts_right, debug = True, execute=False, use_last_img=False):
        ## input two lists of points. Each list contains 3 points. Each point contins
        ## pts_left three transformation
        self.gripper.l_open()
        self.hands.left.open = True
        self.gripper.r_close()
        self.hands.right.open = False

        self.replanner.clear_all_marker()
        
        collision = False

        ##***********************************************************************
        ## generate joint space control of the left
        print("====\nleft hand side trajectory")

        temp_mat = np.copy(self.left_default)
        
        t1 = time.perf_counter()
        temp_mat[:3,3] = np.array(pts_left[0])
        p0 = transformation2pose(temp_mat)
        temp_mat[:3,3] = np.array(pts_left[1])
        p1 = transformation2pose(temp_mat)
        temp_mat[:3,3] = np.array(pts_left[2])
        p2 = transformation2pose(temp_mat)
        ## triangle trjactory
        curve_path_left = [p0, p1, p2, p0]
        # curve_path_left = [p0, p0, p0, p0]

        ##***********************************************************************
        ## generate joint space control of the right
        print("====\nright hand side trajectory")
        ## a good starting position
        temp_mat = np.copy(self.right_default)
        
        temp_mat[:3,3] = np.array(pts_right[0])
        p0 = transformation2pose(temp_mat)
        temp_mat[:3,3] = np.array(pts_right[1])
        p1 = transformation2pose(temp_mat)
        temp_mat[:3,3] = np.array(pts_right[2])
        p2 = transformation2pose(temp_mat)
        ## triangle trjactory
        curve_path_right = [p0, p1, p2, p0]

        t2 = time.perf_counter()
        print("Runtime (prepare points): {}ms".format((t2-t1)*1000))

        ##***********************************************************************
        ## solve the IK of the 6 points
        t1 = time.perf_counter()
        solved_flag, qls = self.ut.pts2qs(0, curve_path_left, [self.j6l_default]*len(curve_path_left))
        if solved_flag < 0:
            print('not enough waypoints for the left arm, skip')
            return -1
        print("sovled q1:{}".format(len(qls)))
        
        solved_flag, qrs = self.ut.pts2qs(1, curve_path_right, [self.j6r_default]*len(curve_path_right))

        if solved_flag < 0:
            print('not enough waypoints for the right arm, skip')
            return -1
        print("sovled q1:{}".format(len(qrs)))

        t2 = time.perf_counter()
        print("Runtime (solving IK): {}ms".format((t2-t1)*1000))

        ##***********************************************************************
        ## interpolation here

        t1 = time.perf_counter()
        ## solution found, interpolation then execute
        n_samples = 10
        dt = 2
        j_traj_left = interpolation(qls, n_samples, self.dt)
        j_traj_right = interpolation(qrs, n_samples, self.dt)
        key_pose_no = [0, n_samples+1, n_samples*2+2, n_samples*3+3]
        # print(key_pose_no)

        t2 = time.perf_counter()
        print("Runtime (interpolation): {}ms".format((t2-t1)*1000))
        ####--------####
        t1 = time.perf_counter()

        # detected_list = self.check_traj_collision(j_traj_left, j_traj_right)
        # print("".format(detected_list))
        # if execute:
        #     self.ut.execute(j_traj_left, j_traj_right)

        # choice = input("press any key to continue")

        ##***********************************************************************
        ## test if two curves will lead to collision
        ## detour if colllision exist in the planned trajectory
        idx_s = key_pose_no[1]+1
        idx_e = key_pose_no[2]
        new_sub_traj_l, new_sub_traj_r = self.replanner.binary_mp(j_traj_left[idx_s:idx_e], j_traj_right[idx_s:idx_e])
        

        t2 = time.perf_counter()
        print("Runtime (binary motion planning): {}ms".format((t2-t1)*1000))

        self.replanner.clear_all_marker()
        
        ####--------####
        t1 = time.perf_counter()
        ## insert to the original trajectory
        j_traj_left  = j_traj_left[:idx_s-1]  + new_sub_traj_l + j_traj_left[idx_e+1:]
        j_traj_right = j_traj_right[:idx_s-1] + new_sub_traj_r + j_traj_right[idx_e+1:]

        detected_list = self.replanner.check_traj_collision(new_sub_traj_l, new_sub_traj_r)

        t2 = time.perf_counter()
        print("Runtime (check trajectory collision): {}ms\n".format((t2-t1)*1000))
        print("".format(detected_list))

        # if collision:
        #     print("**** Possible collision detected! Stop! ****")
        #     return -1


        ##***********************************************************************
        if execute:
            t1 = time.perf_counter()
            self.ut.execute(j_traj_left, j_traj_right)
            t2 = time.perf_counter()
            print("Runtime (executing): {}ms\n".format((t2-t1)*1000))
            return 0

        print("Execution done!")

        return -1 ## for any other reason failed to execute

    def wrapping(self, execute=True):
        self.gripper.l_open()
        self.gripper.r_open()
        print("GOTO initial position")
        ## left initial pose
        temp_mat = np.copy(self.left_default)
        temp_mat[:3,3] = np.array([0.35, 0.09, 0.32])

        sp_left = transformation2pose(temp_mat)
        ## qs of the left arm
        solved_flag, qls = self.ut.pts2qs(0, [sp_left], [self.j6l_default])
        print(solved_flag)
        print(len(qls))

        if execute:
            print("left hand go to pose")
            self.j_ctrl.robot_setjoint(qls[0], group=0)

        

        ## right inital pose
        temp_mat = np.copy(self.right_default)
        temp_mat[:3,3] = np.array([0.35,-0.15, 0.15])
        sp_right = transformation2pose(temp_mat)
        ## qs of the right arm
        solved_flag, qrs = self.ut.pts2qs(1, [sp_right], [self.j6r_default])
        print(solved_flag)

        if execute:
            print("right hand go to pose")
            self.j_ctrl.robot_setjoint(qrs[0], group=1)

        input("Press enter after get the rope ready")


        ## wrapping for one or more rounds
        self.gripper.r_close()
        y_left =0.09 # with collision
        # y_left =0.11 # no collision
        self.step([[0.35, y_left, 0.32],[0.4, y_left, 0.17],[0.3, y_left,0.17]],\
                  [[0.35,  -0.15, 0.15],[0.3,  -0.15, 0.17],[0.4, -0.15, 0.17]], execute=execute)
        self.gripper.r_open()

        return

if __name__ == '__main__':
    run = True
    menu  = '=========================\n'
    menu += '1. Reset the robot\n'
    menu += '2. Demo\n'
    menu += "3. Reset the robot's left arm pose\n"
    menu += '4. Clear all markers\n'
    menu += '0. Exit\n'
    menu += 'Your input:'

    rospy.init_node('wrap_wrap', anonymous=True)
    # rate = rospy.Rate(10)
    rospy.sleep(1)

    rw = robot_wrapping()
    while run:
        choice = input(menu)
        
        if choice in ['1', '2', '3', '4']:
            if choice == '1':
                ## reset the robot
                rw.ut.reset()
            elif choice == '2':
                ## start a new learning, tuning parameters automatically
                '''step(self, pts_left, pts_right, debug = True, execute=False, use_last_img=False)'''
                rw.wrapping()
            elif choice == '3':
                ## reset the robot
                rw.ut.reset_left()
            elif choice == '4':
                rw.replanner.clear_all_marker()

        else:
            ## exit
            run = False
