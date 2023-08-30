#! /usr/bin/env python

import rospy
from visualization_msgs.msg import Marker

class marker():
  def __init__(self):
    self.pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 10)

  def show(self, pose):
    marker = Marker()

    marker.header.frame_id = "world"
    marker.header.stamp = rospy.Time.now()

    # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
    marker.type = Marker.SPHERE
    marker.id = 0

    # Set the scale of the marker
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05

    # Set the color
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 0.5

    # Set the pose of the marker
    marker.pose = pose

    self.pub.publish(marker)


if __name__ == '__main__':
  rospy.init_node('rviz_marker')
  # while not rospy.is_shutdown():
  goal = marker()
  
  rospy.sleep(1.0)
  goal.show(x=1, y=1, z=1)
    # rospy.rostime.wallsleep(1.0)