#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy
from lab5.srv import Oint
from geometry_msgs.msg import PoseStamped

from tf.transformations import *

from params import get_parameters

freq = 50

#make axes show at the beginning
def init():
    robot_pose = PoseStamped()
    robot_pose.header.frame_id = "base_link"
    robot_pose.header.stamp = rospy.Time.now()
    robot_pose.pose.position.x = a2
    robot_pose.pose.position.y = 0
    robot_pose.pose.position.z = a1
    robot_pose.pose.orientation.x = 0
    robot_pose.pose.orientation.y = 0
    robot_pose.pose.orientation.z = 0
    robot_pose.pose.orientation.w = 1

    pub.publish(robot_pose)

def interpolate(data):
    if(data.time < 0):
        return ("Czas musi być większy od zera")
    
    global start
 
    end = [data.tx, data.ty, data.tz, data.rx, data.ry, data.rz]
 	
    change = start

    step=[
        (end[0]-start[0])/(freq*data.time),
        (end[1]-start[1])/(freq*data.time),
        (end[2]-start[2])/(freq*data.time), 
        (end[3]-start[3])/(freq*data.time), 
        (end[4]-start[4])/(freq*data.time),
        (end[5]-start[5])/(freq*data.time)
    ]

    for k in range(0, int(freq*data.time)+1):
        for i in range(0, 6):
            change[i]=change[i]+step[i]

        tx=change[0]
        ty=change[1]
        tz=change[2]
        rx=change[3]
        ry=change[4]
        rz=change[5]

        start=[tx, ty, tz, rx, ry, rz]

        tm = identity_matrix()
        rzm = rotation_matrix(rz, zaxis)
        rym = rotation_matrix(ry, yaxis)
        rxm = rotation_matrix(rx, xaxis)
        end_matrix = concatenate_matrices(rxm, rym, rzm, tm)

        xq, yq, zq, wq = quaternion_from_matrix(end_matrix)
  
        robot_pose = PoseStamped()
        robot_pose.header.frame_id = "base_link"
        robot_pose.header.stamp = rospy.Time.now()
        robot_pose.pose.position.x = tx
        robot_pose.pose.position.y = ty
        robot_pose.pose.position.z = tz
        robot_pose.pose.orientation.x = xq
        robot_pose.pose.orientation.y = yq
        robot_pose.pose.orientation.z = zq
        robot_pose.pose.orientation.w = wq
   
        rate = rospy.Rate(freq)
        pub.publish(robot_pose)
        rate.sleep()

    start = end
    return (str(data.tx)+" "+str(data.ty)+" "+str(data.tz)+" "+str(data.rx)+" "+str(data.ry)+" "+str(data.rz))

if __name__ == "__main__":

    a1 = 0.7
    a2 = 1

    if get_parameters('dlugosc1'):
        a1 = rospy.get_param("/dlugosc1")

    if get_parameters('dlugosc2'):
        a2 = rospy.get_param("/dlugosc2")   

    start = [ a2, 0, a1, 0, 0, 0]
    xaxis, yaxis, zaxis = (1, 0, 0), (0, 1, 0), (0, 0, 1)

    rospy.init_node('oint_srv')
    pub = rospy.Publisher('OintAxes',PoseStamped, queue_size=10)
    s = rospy.Service('oint', Oint, interpolate)
    rospy.sleep(0.5)
    init()
    rospy.spin()
