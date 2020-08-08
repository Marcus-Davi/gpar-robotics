#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#implementar um controle que mande o robo ir da posição atual para a posição desejada
import rospy
from geometry_msgs.msg import Twist, Pose2D, Vector3
from turtlesim.msg import Pose
import numpy as np
from math import pi, atan2, sqrt, cos, sin, tan
import sys
import time

position = Pose2D()
position.x = 0
position.y = 0
position.theta = 0

l = 0.38    #dados do Prioneer do coppelia
r = 0.185/2 #dados do Prioneer do coppelia

def go2goal(xgoal = 0, ygoal = 0, thetagoal = 0):
    rospy.init_node('hulk_controller', anonymous=True)

    #COPPELIA
    velocity_publisher = rospy.Publisher('/velocidade_hulk', Twist, queue_size=10)
    vel_subscriber = rospy.Subscriber('/measured_velocity', Vector3, updatePose)

    #TARTARUGA DO ROS
    #velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) 
    #pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, updatePose)

    goal = Pose2D()
    goal.x = xgoal
    goal.y = ygoal
    goal.theta = thetagoal


    erro_acumulado = 0
    tol = 0.05
    v = 1
    w = 0

    #PID {
    kp = 5
    ki = 0.00000000000001
    #kd = 0
    #    }

    while(v > tol):
        errox = goal.x - position.x 
        erroy = goal.y - position.y     
        
        ux = errox
        uy = erroy

        theta_r = atan2(uy, ux)

        v = sqrt(ux**2 + uy**2)

        errot = theta_r - position.theta # erro do angulo theta

        erro_acumulado += errot 

        w = kp * errot + ki *  erro_acumulado #PI
        vel = Twist()

        vel.linear.x = v
        vel.linear.y = 0
        vel.linear.z = 0

        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = w

        #rospy.loginfo(vel)
        velocity_publisher.publish(vel)

        #updates
        past_erro = errot
        
    pose_subscriber.unregister()
    return



#USO COM A TARTARUGA DO ROS
def updatePose(pose):
    '''
    rospy.loginfo(pose)
    position.x = pose.x
    position.y = pose.y
    position.theta = pose.theta
'''

#USO COM O SIMULADOR DO VREṔ
def updatePose(weel_vel):
    #rospy.loginfo(weel_vel)
    ve = weel_vel.x
    vd = weel_vel.y
    
    v = r * (vd + ve)/2
    w = (vd - ve)/l
    
    dt = 0.1

    position.x = position.x + v*cos(position.theta) * dt
    position.y = position.y + v*sin(position.theta) * dt
    position.theta = position.theta + w*dt
    position.theta = atan2(tan(position.theta), 1)
    rospy.loginfo(position)
 

if __name__ == "__main__":
    try:
        x = 5
        y = 5
        theta = pi/2
        go2goal(x,y,theta)

    except rospy.ROSInterruptException :
        pass