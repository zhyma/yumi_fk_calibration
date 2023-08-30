#!/usr/bin/env python3

import sys

from math import pi

from trac_ik_python.trac_ik import IK
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64,Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list

import actionlib
from control_msgs.msg import  FollowJointTrajectoryAction, FollowJointTrajectoryGoal

class joint_ctrl():

    def __init__(self, ctrl_group):
        self.ctrl_group = ctrl_group
        self.trajectory_pub = rospy.Publisher('/yumi/joint_traj_pos_controller_both/command', JointTrajectory, queue_size=10)
        self.client = actionlib.SimpleActionClient('/yumi/joint_traj_pos_controller_both/follow_joint_trajectory', FollowJointTrajectoryAction)

        self.client.wait_for_server()

    def robot_reset(self):
        # self.robot_default_l()
        # self.robot_default_r()
        joints_val = [-1.1694, -2.3213, 1.1694, -0.3665, 0.2792, 1.2392, -0.3491,\
                       1.1694, -2.3213, -1.1694, -0.3665, 0.2792, 1.2392, -0.3491]
        self.robot_setjoint(joints_val)

    def robot_setjoint(self, value, group=-1):
        all_value = []
        if (len(value) == 7) and (group==0):
            # left arm
            l_joints_val = value
            r_joints_val = self.ctrl_group.get_current_joint_values()[7:]
            all_value = value + r_joints_val
            
        elif (len(value) == 7) and (group==1):
            # right arm
            l_joints_val = self.ctrl_group.get_current_joint_values()[0:7]
            r_joints_val = value
            all_value = l_joints_val + value

        elif len(value)==14:
            # set for all joints, len(value) == 14
            all_value = value
        
        else:
            all_value = self.ctrl_group.get_current_joint_values()

        self.ctrl_group.set_joint_value_target(all_value)
        self.ctrl_group.go(wait=True)
        self.ctrl_group.stop()


    def robot_default_l_low(self):
        # l_joints_val = [ -1.1694, -2.3213, 1.1694, -0.3665, 0.2792, 1.2392, -0.3491]
        # r_joints_val = self.ctrl_group.get_current_joint_values()[7:]
        # self.robot_setjoint(l_joints_val+r_joints_val)
        self.robot_setjoint([ -1.1694, -2.3213, 1.1694, -0.3665, 0.2792, 1.2392, -0.3491], group=0)

    def robot_default_r_low(self):
        # l_joints_val = self.ctrl_group.get_current_joint_values()[0:7]
        # r_joints_val = [ 1.1694, -2.3213, -1.1694, -0.3665, 0.2792, 1.2392, -0.3491]
        # self.robot_setjoint(l_joints_val+r_joints_val)
        self.robot_setjoint([ 1.1694, -2.3213, -1.1694, -0.3665, 0.2792, 1.2392, -0.3491], group=1)

    def exec(self, value_list, dt, start_time=0):
        goal =  FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['yumi_joint_1_l','yumi_joint_2_l','yumi_joint_7_l','yumi_joint_3_l','yumi_joint_4_l','yumi_joint_5_l','yumi_joint_6_l',\
                                       'yumi_joint_1_r','yumi_joint_2_r','yumi_joint_7_r','yumi_joint_3_r','yumi_joint_4_r','yumi_joint_5_r','yumi_joint_6_r']

        for i in range(len(value_list)):
            point = JointTrajectoryPoint()
            point.positions = value_list[i]
            point.time_from_start = rospy.Duration(start_time+dt*i)

            goal.trajectory.points.append(point)

        self.client.send_goal(goal)
        self.client.wait_for_result()

        return self.client.get_result()


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('yumi_test', anonymous=True)
    robot = moveit_commander.RobotCommander()

    print(robot.get_group_names())
    scene = moveit_commander.PlanningSceneInterface()

    ctrl_group = moveit_commander.MoveGroupCommander('both_arms')


    j_ctrl = joint_ctrl(ctrl_group)
    print(j_ctrl.ctrl_group.get_current_joint_values())
    # print(j_ctrl.ctrl_group.get_current_joint_values()[0:7]) #left
    # print(j_ctrl.ctrl_group.get_current_joint_values()[7:])  #right
    j_ctrl.robot_reset()