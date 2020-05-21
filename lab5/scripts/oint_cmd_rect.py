#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import rospy
import rospkg
from lab5.srv import Oint
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from geometry_msgs.msg import PoseStamped
from math import atan, sin, cos, pi

from params import get_parameters



def init(x,y, z):
    trajectory(x,y,z,3)
    
def kwadrat(x0,y0, z0, x):

    trajectory(x0,y0+x,z0, 5)
    trajectory(x0,y0+x,z0+x, 5)
    trajectory(x0,y0-x,z0+x, 5)
    trajectory(x0,y0-x,z0, 5)
    
 
 
def trajectory(x, y, z, time):
    rospy.wait_for_service('oint')
    try:
        trajektoria=rospy.ServiceProxy('oint', Oint)
        resp1=trajektoria(x,y,z,0,0,0,time)
        return resp1
    except rospy.ServiceException, e:
        print ("Service call failed: ",e)
    

if __name__ == '__main__':
    
    a1 = 0.7
    a2 = 1
    if get_parameters('dlugosc1'):
        a1 = rospy.get_param("/dlugosc1")

    if get_parameters('dlugosc2'):
        a2 = rospy.get_param("/dlugosc2")
    init(a2, 0, a1)
    while not rospy.is_shutdown():
        rospy.init_node('rect', anonymous=False)
        kwadrat(a2, 0, a1, 0.4)
