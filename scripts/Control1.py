#!/usr/bin/env python

import rospy
from time import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
from math import *
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler

CONST_LINEAR_SPEED_FORWARD = 0.08
CONST_ANGULAR_SPEED_FORWARD = 0.0
CONST_LINEAR_SPEED_TURN = 0.06
CONST_ANGULAR_SPEED_TURN = 0.4


K_RO = 2
K_ALPHA = 15
K_BETA = -3
V_CONST = 0.1 # [m/s]

# K
GOAL_DIST_THRESHOLD = 0.1 # [m]
GOAL_ANGLE_THRESHOLD = 15 # [degrees]

# Get theta radiann
def getRotation(odomMsg):
    # orientation_q = odomMsg.pose.pose.orientation
    # orientation_list = [ orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    # (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    yaw=odomMsg.pose.theta
    return yaw

# Get vi tri  x y
def getPosition(odomMsg):
    x = odomMsg.pose.x
    y = odomMsg.pose.y
    return ( x , y)

# Get toc do dai
def getLinVel(odomMsg):
    return odomMsg.linear

# Get toc do goc 
def getAngVel(odomMsg):
    return odomMsg.angular

# Gui den twist
def createVelMsg(v,w):
    velMsg = Twist()
    velMsg.linear = v
    # velMsg.linear.y = 0
    # velMsg.linear.z = 0
    # velMsg.angular.x = 0
    # velMsg.angular.y = 0
    velMsg.angular = w
    return velMsg

# truoc
def robotGoForward(velPub):
    velMsg = createVelMsg(CONST_LINEAR_SPEED_FORWARD,CONST_ANGULAR_SPEED_FORWARD)
    velPub.publish(velMsg)

# re trai
def robotTurnLeft(velPub):
    velMsg = createVelMsg(CONST_LINEAR_SPEED_TURN,+CONST_ANGULAR_SPEED_TURN)
    velPub.publish(velMsg)

# Re phai
def robotTurnRight(velPub):
    velMsg = createVelMsg(CONST_LINEAR_SPEED_TURN,-CONST_ANGULAR_SPEED_TURN)
    velPub.publish(velMsg)

# Stop command
def robotStop(velPub):
    velMsg = createVelMsg(0.0,0.0)
    velPub.publish(velMsg)



# Thuc hien hanh dong
def robotDoAction(velPub, action):
    status = 'robotDoAction => OK'
    if action == 0:
        robotGoForward(velPub)
    elif action == 1:
        robotTurnLeft(velPub)
    elif action == 2:
        robotTurnRight(velPub)
    else:
        status = 'robotDoAction => INVALID ACTION'
        robotGoForward(velPub)

    return status

# Thuat toan dieu khien hoi tiep 
def robotFeedbackControl(velPub, x, y, theta, x_goal, y_goal, theta_goal):
    # theta goal normalization
    if theta_goal >= pi:
        theta_goal_norm = theta_goal - 2 * pi
    else:
        theta_goal_norm = theta_goal

    ro = sqrt( pow( ( x_goal - x ) , 2 ) + pow( ( y_goal - y ) , 2) )
    lamda = atan2( y_goal - y , x_goal - x )

    alpha = (lamda -  theta + pi) % (2 * pi) - pi
    beta = (theta_goal - lamda + pi) % (2 * pi) - pi

    if ro < GOAL_DIST_THRESHOLD and degrees(abs(theta-theta_goal_norm)) < GOAL_ANGLE_THRESHOLD:
        status = 'Goal position reached!'
        v = 0
        w = 0
        v_scal = 0
        w_scal = 0
    else:
        status = 'Goal position not reached!'
        v = K_RO * ro
        w = K_ALPHA * alpha + K_BETA * beta
        v_scal = v / abs(v) * V_CONST
        w_scal = w / abs(v) * V_CONST

    velMsg = createVelMsg(v_scal, w_scal)
    velPub.publish(velMsg)

    return status

def check_stability(k_rho, k_alpha, k_beta):
    return k_rho > 0 and k_beta < 0 and k_alpha > k_rho


def check_strong_stability(k_rho, k_alpha, k_beta):
    return k_rho > 0 and k_beta < 0 and k_alpha + 5 * k_beta / 3 - 2 * k_rho / np.pi > 0
if __name__ == '__main__':
    try:
        rospy.init_node('test_odom',anonymous = False)
        rate = rospy.Rate(10)
        print("Start running ")
        
        while not rospy.is_shutdown():
            odomMsg = rospy.wait_for_message('/mcu_pose', Odometry)
            ( x,y ) = getPosition(odomMsg)
            theta = getRotation ( odomMsg )
            print( x, y,theta )
    except rospy.ROSInterruptException:
        print('Terminated!')
        pass