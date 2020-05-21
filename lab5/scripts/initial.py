#!/usr/bin/env python

import rospy
from lab5.srv import Jint

if __name__ == "__main__":
    rospy.init_node('initial', anonymous=True)
    rospy.wait_for_service('jint')
    rate = rospy.Rate(2)
    rate.sleep()
    
    try:
        jint = rospy.ServiceProxy('jint', Jint)
        resp1 = jint(0, 0, 0, 0.1, 'linear')
        print(resp1.status)
    except rospy.ServiceException, e:
        print ("Service call failed: %s"%e)
