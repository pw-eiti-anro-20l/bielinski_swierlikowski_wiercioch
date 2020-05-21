#!/usr/bin/env python
import rospy

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

from sensor_msgs.msg import JointState
path = Path()

def callback(data):
    global path
    path.header = data.header
    pose = PoseStamped()
    pose.header = data.header
    pose.pose = data.pose
    path.poses.append(pose)
    path_pub.publish(path)



if __name__ == '__main__':
    rospy.init_node('PathNode', anonymous=False)

    odom_sub = rospy.Subscriber('KdlAxes', PoseStamped, callback)
    path_pub = rospy.Publisher('/JintPath', Path, queue_size=10)
    rospy.spin()