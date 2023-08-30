'''
replan trajectories that contain collision
'''
import sys, copy, pickle, os, time, rospy, moveit_commander, cv2
sys.path.append('../../')

from utils.workspace_tf          import workspace_tf, pose2transformation, transformation2pose
from utils.robot.yumi_fk         import yumi_fk
from utils.robot.interpolation   import interpolation

from visualization_msgs.msg      import Marker, MarkerArray


class replanner():
    def __init__(self, yumi, hands, dt):
        self.yumi = yumi
        self.hands = hands
        self.dt = dt
        self.marker_array = MarkerArray()
        self.marker_pub = rospy.Publisher("/collision", MarkerArray, queue_size = 1000)

    def check_traj_collision(self, traj_left, traj_right):
        '''
        input: left and right trajectory
        output: detected_list is a list, each element contain [No. of the colliding pose, distance]
                both No. and distance are scalar.
        '''
        detected_list = []
        for i in range(len(traj_left)):
            # test each point in the joint space
            link7_l = yumi_fk(traj_left[i],  side=0)[-1]
            link7_r = yumi_fk(traj_right[i], side=1)[-1]
            collide, d = self.hands.check_collision(link7_l, link7_r, marker_array=self.marker_array, render_all=False)
            self.marker_pub.publish(self.marker_array)
            if len(collide) > 0:
                detected_list.append([i, d])
        
        return detected_list


    def joint_replan(self, joints_l, joints_r, d):
        ## FK get current pose
        link7_l_ht = yumi_fk(joints_l, side=0)[-1]
        link7_r_ht = yumi_fk(joints_r, side=1)[-1]
        ## move the current pose accordingly
        ## left arm: move to y+ for d/2
        ## right arm: move to y+ for -d/2
        link7_l_ht[2,3] += d/2
        link7_r_ht[2,3] -= d/2
        ## IK to get the new joints configuration
        pose_left = transformation2pose(link7_l_ht)
        pose_right = transformation2pose(link7_r_ht)
        ql = self.yumi.ik(0, pose_left, with_restrict=True, joint_6_value=joints_l[-1])
        qr = self.yumi.ik(1, pose_right, with_restrict=True, joint_6_value=joints_r[-1])

        return ql, qr

    def binary_mp(self, traj_l, traj_r):
        detected_list = self.check_traj_collision(traj_l, traj_r)
        print(detected_list)
        if len(detected_list) > 0:
            replan_idx = (detected_list[0][0] + detected_list[-1][0])//2
            d = max([i[1] for i in detected_list])

            # print("replan_idx is {},\n traj_l[i]:{}\n traj_r[i]:{}\n d: {}".format(replan_idx, traj_l[replan_idx], traj_r[replan_idx], d))

            n1 = replan_idx - 1
            ## change d here gradually 
            print("collision detected, maximum invasion: {}".format(d))
            joint_l, joint_r = self.joint_replan(traj_l[replan_idx], traj_r[replan_idx], d)
            if n1 > 0:
                ## redo interpolation here
                re_int_sub1_l = interpolation([traj_l[0], joint_l], n1, self.dt)
                re_int_sub1_r = interpolation([traj_r[0], joint_r], n1, self.dt)
                ## check the sub trajectory
                sub_traj1_l, sub_traj1_r = self.binary_mp(re_int_sub1_l, re_int_sub1_r)
            else:
                sub_traj1_l = [traj_l[0], joint_l]
                sub_traj1_r = [traj_r[0], joint_r]

            n2 = len(traj_l) - replan_idx - 2
            if n2 > 0:
                re_int_sub2_l = interpolation([joint_l, traj_l[-1]], n2, self.dt)
                re_int_sub2_r = interpolation([joint_r, traj_r[-1]], n2, self.dt)
                sub_traj2_l, sub_traj2_r = self.binary_mp(re_int_sub2_l, re_int_sub2_r)
            else:
                sub_traj2_l = [joint_l, traj_l[-1]]
                sub_traj2_r = [joint_r, traj_r[-1]]

            return sub_traj1_l + sub_traj2_l[1:], sub_traj1_r + sub_traj2_r[1:]

            # re_int_sub1_l = interpolation([traj_l[0], joint_l], n1, self.dt)
            # re_int_sub1_r = interpolation([traj_r[0], joint_r], n1, self.dt)
            # return re_int_sub1_l +[traj_l[-1]], re_int_sub1_r +[traj_r[-1]]

        else:
            return traj_l, traj_r

    def clear_all_marker(self):
        marker = Marker()
        marker.id = 0
        marker.action = Marker.DELETEALL
        self.marker_array.markers.clear()
        self.marker_array.markers.append(marker)
        self.marker_pub.publish(self.marker_array)
        rospy.sleep(0.2)

if __name__=="__main__":
    ...