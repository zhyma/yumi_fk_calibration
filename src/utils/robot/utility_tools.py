'''
replan trajectories that contain collision
'''
import sys, copy, pickle, os, time, rospy, moveit_commander, cv2
sys.path.append('../../')

from utils.workspace_tf          import workspace_tf, pose2transformation, transformation2pose
from utils.robot.yumi_fk         import yumi_fk
from utils.robot.interpolation   import interpolation

from visualization_msgs.msg      import Marker, MarkerArray

def tf_with_offset(tf, offset):
    rot = tf[:3, :3]
    new_tf = copy.deepcopy(tf)
    d_trans = np.dot(rot, np.array(offset))
    for i in range(3):
        new_tf[i, 3]+=d_trans[i]

    return new_tf

def pose_with_offset(pose, offset):
    ## the offset is given based on the current pose's translation
    tf = pose2transformation(pose)
    new_tf = tf_with_offset(tf, offset)
    new_pose = transformation2pose(new_tf)

    ## make sure it won't go too low
    if new_pose.position.z < 0.03:
            new_pose.position.z = 0.03
    return new_pose

class utility_tools():
    def __init__(self, yumi, j_ctrl, gripper):
        self.yumi = yumi
        self.j_ctrl = j_ctrl
        self.gripper = gripper

    def move2pt(self, point, j6_value, group = 0):
        q = self.yumi.ik(group, point, with_restrict=True, joint_6_value=j6_value)
        if type(q) is int:
            print('No IK found')
            return -1
        else:
            self.j_ctrl.robot_setjoint(group, q)

    def pts2qs(self, group, pts, j6_list):
        '''
        input:
            - group: 0 for left and 1 for right
            - pts: a list of poses that needs to solved by IK
            - j6_list: a list of the preset j6 value, should have the same length as pts
        output:
            - flag: -1 is not solved, 0 is solved
            - if flag is 0, the second parameter is the list of solved joints corresponding to the 
        '''
        no_ik_found = []
        n_pts = len(pts)
        q_knots = []
        if len(j6_list) == len(pts):
            ## solve each workspace point with the wrist joint defined
            for i in range(n_pts):
                # print('point {} is\n{}\n{}'.format(i, pts[i], last_j_angle))
                q = self.yumi.ik(group, pts[i], with_restrict=True, joint_6_value=j6_list[i])
                if q==-1:
                    ## no IK solution found, remove point
                    no_ik_found.append(i)
                else:
                    q_knots.append(q)

        else:
            print("Do IK without the joint angle of the last joint defined.")
            # without the joint angle of the last joint defined.
            for i in range(n_pts):
                # print('point {} is\n{}\n{}'.format(i, pts[i], last_j_angle))
                q = self.yumi.ik(0, pts[i], with_restrict=False)
                ## reversed searching, therefore use -d_j
                if q==-1:
                    ## no IK solution found, remove point
                    no_ik_found.append(i)
                else:
                    # print("point {} solved".format(i))
                    q_knots.append(q)


        if len(no_ik_found) > 0:
            print("No IK solution is found at point: ",end='')
            for i in no_ik_found:
                print(str(i), end=',')
            print("\n")
            return -1, no_ik_found

        return 0, q_knots

    def execute(self, j_traj_left, j_traj_right):
        # # Move to the initial pose
        # self.j_ctrl.exec([j_traj_left[0] + j_traj_right[0]], 0.2)
        # # rospy.sleep(2)
        # # # self.gripper.l_close()
        # # # self.hands.left.open = False
        # # rospy.sleep(0.8)

        # executing the whole trajectory
        two_trajs = []
        for i in range(len(j_traj_left)):
            two_trajs.append(j_traj_left[i] + j_traj_right[i])

        self.j_ctrl.exec(two_trajs, 0.2)

        # # make sure it reaches the last pose
        # self.j_ctrl.exec([j_traj_left[-1] + j_traj_right[-1]], 0.2)
        return 0 ## execute successfully (hopefully)

    def reset(self):
        ##-------------------##
        ## reset the robot
        print('reset the robot pose')
        self.gripper.l_open()
        rospy.sleep(1)
        self.gripper.r_open()
        rospy.sleep(1)
        self.j_ctrl.robot_default_l_low()
        rospy.sleep(1)
        self.j_ctrl.robot_default_r_low()

        self.gripper.l_open()
        self.gripper.r_open()

        rospy.sleep(3)
        print('reset done')

    def reset_left(self):
        ##-------------------##
        ## reset the robot
        print("reset the robot's left hand pose")

        self.gripper.l_open()
        rospy.sleep(1)
        self.j_ctrl.robot_default_l_low()

        self.gripper.l_open()

        rospy.sleep(3)
        print('reset done')


if __name__=="__main__":
    ...