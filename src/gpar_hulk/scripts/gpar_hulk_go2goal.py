#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#implementar um controle que mande o robo ir da posição atual para a posição desejada
import rospy
from geometry_msgs.msg import Twist, Pose2D
from turtlesim.msg import Pose
import numpy as np
from math import pi, atan2, sqrt
import sys

position = Pose2D()
position.x = 0
position.y = 0
position.theta = 0



def go2goal(xgoal = 0, ygoal = 0, thetagoal = 0):
    rospy.init_node('hulk_controller', anonymous=True)
    #velocity_publisher = rospy.Publisher('/velocidade_hulk', Twist, queue_size=10)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, updatePose)

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
    ki = 0.000001
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

def updatePose(pose):
    rospy.loginfo(pose)
    position.x = pose.x
    position.y = pose.y
    position.theta = pose.theta
    pass

if __name__ == "__main__":
    try:
        x = 8.5
        y = 8.5
        theta = pi/2
        go2goal(x,y,theta)

    except rospy.ROSInterruptException :
        pass