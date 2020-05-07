#!/usr/bin/python
# -*- coding: utf-8 -*-

import json
import rospy
import rospkg
from sensor_msgs.msg import JointState
from tf.transformations import *
from geometry_msgs.msg import PoseStamped

def get_parameters(param_name):
    if rospy.has_param(param_name):
        ros_param_name='/'+param_name
        param_value=rospy.get_param(ros_param_name)
        param_value_string=str(param_value)
        
        if param_value>=0 and isinstance(param_value, (int, float)):
            #print("Succesful initialisation of parameter: " + ros_param_name)
            return True
        else:
            
            #print("Unsuccesful initialisation of parameter: " + ros_param_name)
            #print("Length value must be a positive number")
            return False
    else:
        #print("There is no parameter called /" + param_name + ". Proceeding with default length value.")
        return False



def callback(data):
    
    main_matrix = translation_matrix((0, 0, 0));
    
    
    i=0
    for joint in dh_file:
        name = joint['name']
        a = joint["a"]
        d = joint["d"]
        al = joint["al"]
        th=joint['th']
        

        if name == 'i3' and get_parameters('dlugosc1'):
        	a = rospy.get_param("/dlugosc1")

        if name == 'hand' and get_parameters('dlugosc2'):
            a = rospy.get_param("/dlugosc2")

        matrixD = translation_matrix((0, 0, d))
        if name != 'hand':
            pos=data.position[i]
            if i>=0:
                if restrictions[i]['backward']>pos:
                    pos = restrictions[i]['backward']
                    warn_string="[NONKDL] Przekroczono ograniczenie dolne stawu o numerze: "+str(i+1)
                    rospy.logerr(warn_string)
                elif restrictions[i]['forward']<pos:
                    pos = restrictions[i]['forward']
                    warn_string="[NONKDL] Przekroczono ograniczenie gorne stawu o numerze: "+str(i+1)
                    rospy.logerr(warn_string)
            matrixTheta = rotation_matrix(th+pos, z_axis)
        else:
            matrixTheta = rotation_matrix(th, z_axis)
        matrixA = translation_matrix((a, 0, 0))
        matrixAlpha = rotation_matrix(al, x_axis)

        trans_matrix = concatenate_matrices(matrixA,matrixAlpha,matrixTheta, matrixD)
        main_matrix = concatenate_matrices(main_matrix, trans_matrix)
        i = i + 1
        
    
    x, y, z = translation_from_matrix(main_matrix)
    qx, qy, qz, qw = quaternion_from_matrix(main_matrix)
    non_kdl_pose = PoseStamped()
    non_kdl_pose.header.frame_id = 'base_link'
    non_kdl_pose.header.stamp = rospy.Time.now()
    
    non_kdl_pose.pose.position.x = x
    non_kdl_pose.pose.position.y = y
    non_kdl_pose.pose.position.z = z
    non_kdl_pose.pose.orientation.x = qx
    non_kdl_pose.pose.orientation.y = qy
    non_kdl_pose.pose.orientation.z = qz
    non_kdl_pose.pose.orientation.w = qw

    pub.publish(non_kdl_pose)
    #print(data)

if __name__ == '__main__':
    
    x_axis = (1,0,0)
    y_axis = (0,1,0)
    z_axis = (0,0,1)

    rospack = rospkg.RosPack()
    
    rospy.init_node('NONKDL_DKIN', anonymous=False)
    dh_file ={}
    restr_file={}
    
    with open(rospack.get_path('lab4') + '/dh_parameters.json', 'r') as file:
        dh_file= json.loads(file.read())
    
    with open(rospack.get_path('lab4') + '/restrictions.json', 'r') as file:
        restrictions= json.loads(file.read())
    
    
    pub = rospy.Publisher('NoKdlAxes', PoseStamped, queue_size=10)
    rospy.Subscriber("joint_states", JointState , callback)
    
    rospy.spin()
