#! /usr/bin/env python

import rospy
from time import time
from time import sleep
from datetime import datetime

import sys
DATA_PATH = '/home/le/catkin_ws/src/agv1/Data'
MODULES_PATH = '/home/le/catkin_ws/src/agv1/scripts'
sys.path.insert(0, MODULES_PATH)

from Reinforcement import *
from Lidar_ import *
from Control1 import *

# Real robot
REAL_ROBOT = True

#thoi gian giua 2 hanh dong
MIN_TIME_BETWEEN_ACTIONS = 0.0


# Vi tri muc tieu
if REAL_ROBOT:
    X_INIT = 0.0
    Y_INIT = 0.0
    THETA_INIT = 0.0
    X_GOAL = 0
    Y_GOAL = 2
    THETA_GOAL = 0
else:
    RANDOM_INIT_POS = False

    X_INIT = 3
    Y_INIT = 2
    THETA_INIT = 40

    X_GOAL = 2
    Y_GOAL = -0.5
    THETA_GOAL = 0

# sourt Qtable
Q_TABLE_SOURCE = DATA_PATH + '/Log_learning_FINAL'

if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous = False)
        rate = rospy.Rate(10)

        #setPosPub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size = 10)
        velPub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10) #node thuc te

        actions = createActions()
        state_space = createStateSpace()
        Q_table = readQTable(Q_TABLE_SOURCE+'/Qtable.csv')
        print('Initial Q-table:')
        print(Q_table)

        # thoi gian bat dau
        t_0 = rospy.Time.now()
        t_start = rospy.Time.now()
        while not (t_start > t_0):
            t_start = rospy.Time.now()

        t_step = t_start
        count = 0

        # robot o vi tri ban dau
        robot_in_pos = True

        sleep(1)

        # main loop
        while not rospy.is_shutdown():
            msgScan = rospy.wait_for_message('/scan', LaserScan)
            
            odomMsg = rospy.wait_for_message('/mcu_pose', mcu_pose)
            print(robot_in_pos)
            # thoi gian toi thieu giua 2 hanh dong
            step_time = (rospy.Time.now() - t_step).to_sec()

            if step_time > MIN_TIME_BETWEEN_ACTIONS:
                t_step = rospy.Time.now()

                if robot_in_pos:
                    robotStop(velPub)
                    # init pos
                    if REAL_ROBOT:
                        ( x_init , y_init , theta_init ) = (0, 0, 0)
                        print(x_init)
                        odomMsg = rospy.wait_for_message('/mcu_pose', mcu_pose)
                        ( x , y ) = getPosition(odomMsg)
                        theta = degrees(getRotation(odomMsg))
                        print(x,y,theta)
                        robot_in_pos = False
                        print('\r\nInitial position:')
                        print('x = %.2f [m]' % x)
                        print('y = %.2f [m]' % y)
                        print('theta = %.2f [degrees]' % theta)
                        print('')
                   
                else:
                    count = count + 1
                    text = '\r\nStep %d , Step time %.2f s' % (count, step_time)

                    # Get vi tri va huong robot 
                    ( x , y ) = getPosition(odomMsg)
                    theta = getRotation(odomMsg)
                    print(x,y,theta)
                    print(X_GOAL,Y_GOAL,THETA_GOAL)
                    # Get lidar scan
                    ( lidar, angles ) = lidarScan(msgScan)
                    ( state_ind, x1, x2 ,x3 ,x4 ) = scanDiscretization(state_space, lidar)

                    # Check vat can
                    crash = checkCrash(lidar)
                    object_nearby = checkObjectNearby(lidar)
                    goal_near = checkGoalNear(x, y, X_GOAL, Y_GOAL)
                    enable_feedback_control = True

                    # Stop the simulation
                    if crash:
                        robotStop(velPub)
                        rospy.signal_shutdown('End of testing!')
                        text = text + ' ==> Crash! End of simulation!'
                        status = 'Crash! End of simulation!'
                    # THuat toan dieu khien hoi tiep
                    elif enable_feedback_control and ( not object_nearby or goal_near ):
                        status = robotFeedbackControl(velPub, x, y, theta, X_GOAL, Y_GOAL, radians(THETA_GOAL))
                        text = text + ' ==> Feedback control algorithm '
                        if goal_near:
                            text = text + '(goal near)'
                    # Q-learning algorithm
                    else:
                        ( action, status ) = getBestAction(Q_table, state_ind, actions)
                        if not status == 'getBestAction => OK':
                            print('\r\n', status, '\r\n')

                        status = robotDoAction(velPub, action)
                        if not status == 'robotDoAction => OK':
                            print('\r\n', status, '\r\n')
                        text = text + ' ==> Q-learning algorithm'

                    

                    if status == 'Goal position reached!':
                        robotStop(velPub)
                        rospy.signal_shutdown('End of testing!')
                        text = text + '\r\n\r\nGoal position reached! End of simulation!'

                    print(text)

    except rospy.ROSInterruptException:
        robotStop(velPub)
        print('Simulation terminated!')
        pass
