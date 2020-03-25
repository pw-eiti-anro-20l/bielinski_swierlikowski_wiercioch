#!/usr/bin/env python

import rospy
import sys, select, termios, tty
from geometry_msgs.msg import Twist

def initialise():
    if rospy.has_param('steering_keys_list'):
        keys=rospy.get_param("/steering_keys_list")
        print("Succesful key parameters initialisation")
    else:
        print("There is no parameter called '\steering_keys_list'. Proceeding with default steering settings")
        keys=['w', 'a', 's', 'd']
    return keys
def printInstruction(steeringKeys):
    print("Let's move your turtle!")
    print("Steering:")
    print(steeringKeys[0], "- move forward")
    print(steeringKeys[2], "- move backwards")
    print(steeringKeys[1], "- rotate left")
    print(steeringKeys[3], "- rotate right")

#Funkcja wzieta z teleop_twist_keyboard. Dzieki niej nie trzeba zatwierdzac enterem.
def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def move(steeringKeys):
    # Starts a new node
    rospy.init_node('robot_steer', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    
    while not rospy.is_shutdown():
        
        buf=getKey()
        if buf==steeringKeys[0]:
            vel_msg.linear.x = 2.0
        elif buf==steeringKeys[2]:
            vel_msg.linear.x = -2.0
        elif buf ==steeringKeys[1]:
            vel_msg.angular.z = 2.0        
        elif buf == steeringKeys[3]:
            vel_msg.angular.z = -2.0
        #Break if ctrl+C (pozyczone z teleop_twist_keyboard)    
        elif (buf == '\x03'):
            break
        else:
            continue

         #Publish the velocity
        velocity_publisher.publish(vel_msg)
    
        #rate.sleep()  
        #Zeruje wartosci predkosci, zeby w przyszlej petli na pewno obie poczatkowe predkosci byly zerowe 
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
    
    

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)     #POZYCZONE Z teleop_twist_keyboard
    try:
    #Testing our function
        steeringKeys=initialise()
        printInstruction(steeringKeys)
        move(steeringKeys)
    except rospy.ROSInterruptException: 
        pass
