#!/usr/bin/env python3

# modified from https://github.com/ros-planning/moveit_tutorials/blob/kinetic-devel/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py

import sys
import copy
import rospy
import numpy as np
from math import pi,sin,cos,asin,acos, degrees

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from transforms3d import euler
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion

from tf.transformations import quaternion_from_matrix, quaternion_matrix

from trac_ik_python.trac_ik import IK

# from .spiral import path_generator

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

sys.path.append('../../')
from utils.workspace_tf import pose2transformation, transformation2pose

def step_back(group, pose, theta):
    ## input is the value of joint_6
    c = cos(theta)
    s = sin(theta)
    if group == 0:
        ## homogeneous transformation matrix from link_6_l to link_7_l
        ht = np.array([[ c, -s, 0, 0.027],\
                       [ 0,  0, 1, 0.029],\
                       [-s, -c, 0, 0    ],\
                       [ 0,  0, 0, 1    ]])
    else:
        ## homogeneous transformation matrix from link_6_r to link_7_r
        ht = np.array([[ c, -s, 0, 0.027],\
                       [ 0,  0, 1, 0.029],\
                       [-s, -c, 0, 0    ],\
                       [ 0,  0, 0, 1    ]])
    inv_ht = np.linalg.inv(ht)
    t = pose2transformation(pose)
    return transformation2pose(np.dot(t, inv_ht))

class move_yumi():
    def __init__(self, robot, scene, ctrl_group, j_ctrl):
        self.robot = robot
        self.scene = scene
        # self.rate = rate
        self.ctrl_group = ctrl_group
        self.ik_solver = []
        ## self.ik_solver[0]: left  arm from world to link 7, all joints
        self.ik_solver.append(IK("world", "yumi_link_7_l", timeout=0.05, epsilon=1e-3,solve_type="Distance"))
        ## self.ik_solver[1]: right arm from world to link 7, all joints
        self.ik_solver.append(IK("world", "yumi_link_7_r", timeout=0.05, epsilon=1e-3,solve_type="Distance"))

        ## self.ik_solver[2]: left  arm from world to link 6, without joint #6
        self.ik_solver.append(IK("world", "yumi_link_6_l", timeout=0.05, epsilon=1e-3,solve_type="Distance"))
        ## self.ik_solver[3]: right arm from world to link 6, without joint #6
        self.ik_solver.append(IK("world", "yumi_link_6_r", timeout=0.05, epsilon=1e-3,solve_type="Distance"))
        self.j_ctrl = j_ctrl

        lower_bound_l, upper_bound_l = self.ik_solver[0].get_joint_limits()
        new_upper_bound_l = [i for i in upper_bound_l]
        new_upper_bound_l[0] = 0
        self.ik_solver[0].set_joint_limits(lower_bound_l, new_upper_bound_l)
        _, upper_bound_l = self.ik_solver[0].get_joint_limits()
        print("left arm upper_bound updated:", end='')
        print(upper_bound_l)

        ## add floor to the planning scene
        updated = False
        floor_pose = PoseStamped()
        floor_pose.header.frame_id = "world"
        # assign cylinder's pose
        floor_pose.pose.position.x = 1
        floor_pose.pose.position.y = 0
        floor_pose.pose.position.z = 0.0025
        floor_pose.pose.orientation.w = 1
        floor_name = "floor"
        rospy.sleep(1)
        self.scene.add_box(floor_name, floor_pose, size=(2, 1, 0.005))

        # ensuring collision updates are received
        start = rospy.get_time()
        seconds = rospy.get_time()
        timeout = 5
        while (seconds - start < timeout) and not rospy.is_shutdown():
            is_known = floor_name in self.scene.get_known_object_names()

            if is_known:
              print("floor added to the scene")
              return

            rospy.sleep(0.1)
            seconds = rospy.get_time()

    def ik(self, side, pose_goal, with_restrict=False, joint_6_value=-100):
        group = 0
        if with_restrict and joint_6_value > -2*pi:
            ## do IK with restrict of the last joint (joint 6)
            ## and the value of joint 6 is given
            if side == 1 or side == 3:
                ## get the current right arm configuration
                group = 3
                seed_state = self.ctrl_group.get_current_joint_values()[7:]
            else:
                ## get the current left arm configuration
                group = 2
                seed_state = self.ctrl_group.get_current_joint_values()[0:7]

            ## calculate the pose of link6 (step back one joint)
            stepback_pose = step_back(group, pose_goal, joint_6_value)

            x = stepback_pose.position.x
            y = stepback_pose.position.y
            z = stepback_pose.position.z
            qx = stepback_pose.orientation.x
            qy = stepback_pose.orientation.y
            qz = stepback_pose.orientation.z
            qw = stepback_pose.orientation.w

            ik_sol = self.ik_solver[group].get_ik(seed_state[:6], x, y, z, qx, qy, qz, qw)
                # cnt -= 1

            if ik_sol == None:
                return -1
            else:
                return [i for i in ik_sol]+ [joint_6_value]

        else:
            ## do IK without restrict
            if side == 1 or side == 3:
                group = 1
                seed_state = self.ctrl_group.get_current_joint_values()[7:]
            else:
                group = 0
                seed_state = self.ctrl_group.get_current_joint_values()[0:7]

            x = pose_goal.position.x
            y = pose_goal.position.y
            z = pose_goal.position.z
            qx = pose_goal.orientation.x
            qy = pose_goal.orientation.y
            qz = pose_goal.orientation.z
            qw = pose_goal.orientation.w

            ik_sol = self.ik_solver[group].get_ik(seed_state, x, y, z, qx, qy, qz, qw)


            if ik_sol == None:
                return -1
            else:
                return ik_sol
    
    def goto_pose(self, group, pose_goal):
        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        
        print(pose_goal)
        group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        plan = group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        group.clear_pose_targets()

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_traj(self, groups, side, path):
        # path should be a list of Pose()

        (plan, fraction) = groups[side].compute_cartesian_path(
                                        path,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

    def execute_plan(self, plan, group):
        group.execute(plan, wait=True)

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('yumi_test', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    ctrl_group = []
    ctrl_group.append(moveit_commander.MoveGroupCommander('left_arm'))
    ctrl_group.append(moveit_commander.MoveGroupCommander('right_arm'))

    yumi = move_yumi(robot, scene, ctrl_group)

    # pose_goal = geometry_msgs.msg.Pose()
    # q = euler.euler2quat(pi, 0, -pi/2, 'sxyz')
    # pose_goal.position.x = 0.5
    # pose_goal.position.y = 0.1
    # pose_goal.position.z = 0.42
    # pose_goal.orientation.x = q[0]
    # pose_goal.orientation.y = q[1]
    # pose_goal.orientation.z = q[2]
    # pose_goal.orientation.w = q[3]
    # yumi.go_to_pose_goal(yumi.ctrl_group[0], pose_goal)
    
    # print("moving to the starting point")

    # pg = path_generator()
    # path = pg.generate_path()
    # # pg.publish_waypoints(path)

    # rospy.sleep(2)

    # cartesian_plan, fraction = yumi.plan_cartesian_traj(yumi.group_l, path)
    # yumi.execute_plan(cartesian_plan, yumi.group_l)
