#! /usr/bin/env python

import rospy
from time import time
from time import sleep
from datetime import datetime
import matplotlib.pyplot as plt

import sys
# DATA_PATH = '/home/quat/catkin_ws/src/agv/Data'
# MODULES_PATH = '/home/quat/catkin_ws/src/agv/scripts'
#sys.path.insert(0, MODULES_PATH)

from Control1 import *

X_INIT = 0.0
Y_INIT = 0.0
THETA_INIT = 0.0
X_GOAL = 3
Y_GOAL = 2
THETA_GOAL = 15

# init trajectory
X_traj = np.array([])
Y_traj = np.array([])
THETA_traj = np.array([])
X_goal = np.array([])
Y_goal = np.array([])
THETA_goal = np.array([])

# FIle dircetory
#LOG_DIR = DATA_PATH + '/Log_feedback_1'

if __name__ == '__main__':
    try:
        # init nodes
        rospy.init_node('F_control_node', anonymous = False)
        rate = rospy.Rate(10)

        # Khoi tao rostopic 
        #setPosPub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size = 10)
        velPub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)


        #print('\r\n' + text)

        # check dieu kien on dinh
        stab_dict = { True : 'Satisfied!', False : 'Not Satisfied!'}

        print('Stability Condition: ' + stab_dict[check_stability(K_RO, K_ALPHA, K_BETA)])
        print('Strong Stability Condition: ' + stab_dict[check_strong_stability(K_RO, K_ALPHA, K_BETA)])

        # because of the video recording
        sleep(5)

        # main loop
        while not rospy.is_shutdown():

            # Cho doi odomMsg
            odomMsg = rospy.wait_for_message('/mcu_pose', Odometry)

            # Nhan vi tri va huong
            ( x , y ) = getPosition(odomMsg)
            theta = getRotation(odomMsg)
            print( x, y, theta)
            # 	
            X_traj = np.append(X_traj, x)
            Y_traj = np.append(Y_traj, y)
            THETA_traj = np.append(THETA_traj, degrees(theta))
            X_goal = np.append(X_goal, X_GOAL)
            Y_goal = np.append(Y_goal, Y_GOAL)
            THETA_goal = np.append(THETA_goal, THETA_GOAL)

            status = robotFeedbackControl(velPub, x, y, theta, X_GOAL, Y_GOAL, radians(THETA_GOAL))

           
            if status == 'Goal position reached!':
                # stop the robot
                robotStop(velPub)

                

                rospy.signal_shutdown('Goal position reached! End of simulation!')
            #     text = text + '\r\n\r\nGoal position reached! End of simulation!'

            # print(text)

    except rospy.ROSInterruptException:
        robotStop(velPub)
        print('Simulation terminated!')
        pass
