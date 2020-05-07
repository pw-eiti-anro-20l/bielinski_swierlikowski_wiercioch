#!/usr/bin/python
# -*- coding: utf-8 -*-

import json
import rospy
import rospkg

from tf.transformations import *

def get_parameters(param_name):
    if rospy.has_param(param_name):
        ros_param_name='/'+param_name
        param_value=rospy.get_param(ros_param_name)
        param_value_string=str(param_value)
        
        if param_value>=0 and isinstance(param_value, (int, float)):
            print("Succesful initialisation of parameter: " + ros_param_name)
            return True
        else:
            
            print("Unsuccesful initialisation of parameter: " + ros_param_name)
            print("Length value must be a positive number")
            return False
    else:
        print("There is no parameter called /" + param_name + ". Proceeding with default length value.")
        return False

rospack = rospkg.RosPack()
    
with open(rospack.get_path('lab4') + '/dh_parameters.json', 'r') as file:
    dhJson= json.loads(file.read())

x_axis = (1,0,0)
y_axis = (0,1,0)
z_axis = (0,0,1)

with open(rospack.get_path('lab4') + '/urdf_parameters.yaml', 'w') as file:
    for joint in dhJson:
        name = joint['name']
        a = joint['a']
        d = joint['d']
        alpha = joint['al']
        theta = joint['th']

        if name == 'i3' and get_parameters('dlugosc1'):
        	a = rospy.get_param("/dlugosc1")

        if name == 'hand' and get_parameters('dlugosc2'):
            a = rospy.get_param("/dlugosc2")
        
        matrixD= translation_matrix( (0, 0, d) )
        matrixTheta = rotation_matrix( theta, z_axis )
        matrixA = translation_matrix( (a, 0, 0) )
        matrixAlpha = rotation_matrix( alpha, x_axis )
        
        macierz_jednorodna = concatenate_matrices(matrixA,matrixAlpha,matrixTheta, matrixD)
        
        [roll,pitch,yaw] = euler_from_matrix(macierz_jednorodna)
        [x,y,z] = translation_from_matrix(macierz_jednorodna)
        
        file.write(name + ":\n")
        file.write("  j_xyz: "+str(x)+" "+str(y)+" "+str(z)+"\n")
        file.write("  j_rpy: "+str(roll)+' '+str(pitch)+' '+str(yaw)+'\n')
        file.write("  l_xyz: "+str(float(a)*(0.5))+' '+str(0)+' '+str(0)+'\n')
        file.write("  l_len: "+str(a)+'\n')
        
