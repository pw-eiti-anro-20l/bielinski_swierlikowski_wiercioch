#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

def get_parameters(param_name, do_print=False):
    if rospy.has_param(param_name):
        ros_param_name='/'+param_name
        param_value=rospy.get_param(ros_param_name)
        param_value_string=str(param_value)
        
        if param_value>=0 and isinstance(param_value, (int, float)):
            if(do_print):
                print("Succesful initialisation of parameter: " + ros_param_name)
            return True
        else:
            if(do_print):
                print("Unsuccesful initialisation of parameter: " + ros_param_name)
                print("Length value must be a positive number")
            return False
    else:
        if(do_print):
            print("There is no parameter called /" + param_name + ". Proceeding with default length value.")
        return False
