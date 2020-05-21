#!/usr/bin/python
# -*- coding: utf-8 -*-

import json
import rospy
import rospkg
import PyKDL
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
    kdl_chain =PyKDL.Chain()   
    Frame = PyKDL.Frame();

    i = 0
    d=0
    th=0
    for joint in dh_file:
        name = joint['name']
        last_d = d
        last_th = th
        a = joint["a"]
        d = joint["d"]
        al = joint["al"]
        th = joint["th"]

        if name == 'i3' and get_parameters('dlugosc1'):
        	a = rospy.get_param("/dlugosc1")

        if name == 'hand' and get_parameters('dlugosc2'):
            a = rospy.get_param("/dlugosc2")

        #forego first iteration
        if i==0:
            i = i+1
            continue
        
        kdl_chain.addSegment(PyKDL.Segment(PyKDL.Joint(PyKDL.Joint.RotZ), Frame.DH(a, al, last_d, last_th)))
        
        i = i+1
    
    jointDisplacement = PyKDL.JntArray(kdl_chain.getNrOfJoints())
    
    #joint displacements including restrictions
    jointDisplacement[0] = data.position[0]
    jointDisplacement[1] = data.position[1]
    jointDisplacement[2] = data.position[2]

    if(data.position[0] < restrictions[0]['backward']):
        jointDisplacement[0] = restrictions[0]['backward']
        rospy.logerr("[KDL] Przekroczono ograniczenie dolne stawu o numerze: 1")
    elif(data.position[0] > restrictions[0]['forward']):
        jointDisplacement[0] = restrictions[0]['forward']
        rospy.logerr("[KDL] Przekroczono ograniczenie gorne stawu o numerze: 1")

    if(data.position[1] < restrictions[1]['backward']):
        jointDisplacement[1] = restrictions[1]['backward']
        rospy.logerr("[KDL] Przekroczono ograniczenie dolne stawu o numerze: 2")
    elif(data.position[1] > restrictions[1]['forward']):
        jointDisplacement[1] = restrictions[1]['forward']
        rospy.logerr("[KDL] Przekroczono ograniczenie gorne stawu o numerze: 2")

    if(data.position[2] < restrictions[2]['backward']):
        jointDisplacement[2] = restrictions[2]['backward']
        rospy.logerr("[KDL] Przekroczono ograniczenie dolne stawu o numerze: 3")
    elif(data.position[2] > restrictions[2]['forward']):
        jointDisplacement[2] = restrictions[2]['forward']
        rospy.logerr("[KDL] Przekroczono ograniczenie gorne stawu o numerze: 3")

    f_k_solver = PyKDL.ChainFkSolverPos_recursive(kdl_chain)

    frame = PyKDL.Frame()
    f_k_solver.JntToCart(jointDisplacement, frame)
    quatr = frame.M.GetQuaternion()
    
    kdl_pose = PoseStamped()
    kdl_pose.header.frame_id = 'base_link'
    kdl_pose.header.stamp = rospy.Time.now()

    kdl_pose.pose.position.x = frame.p[0]
    kdl_pose.pose.position.y = frame.p[1]
    kdl_pose.pose.position.z = frame.p[2]
    kdl_pose.pose.orientation.x = quatr[0]
    kdl_pose.pose.orientation.y = quatr[1]
    kdl_pose.pose.orientation.z = quatr[2]
    kdl_pose.pose.orientation.w = quatr[3]
    
    pub.publish(kdl_pose)
    #print(data)


if __name__ == '__main__':

    rospack = rospkg.RosPack()
    
    rospy.init_node('KDL_DKIN', anonymous=False)
    dh_file ={}
    restrictions = {}

    with open(rospack.get_path('lab5') + '/dh_parameters.json', 'r') as file:
        dh_file= json.loads(file.read())

    with open(rospack.get_path('lab5') + '/restrictions.json', 'r') as file:
        restrictions = json.loads(file.read())

    pub = rospy.Publisher('KdlAxes', PoseStamped, queue_size=10)
    rospy.Subscriber("joint_states", JointState , callback)
    
    rospy.spin()
