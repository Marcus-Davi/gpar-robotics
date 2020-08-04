#!/usr/bin/env python3

import sys
import rospy
from geometry_msgs.msg import Twist
from gpar_hulk_HDC2450 import *
import math
import time

dist_hulk = None  #averiguar
raio_hulk = None  #averiguar
max_vel = 25000 #rpm
max_vel_ang = 2 * max_vel / dist_hulk

def recieveData(vel):

    v = vel.linear.x * max_vel 
    w = vel.angular.z * max_vel_ang

    vd_rad = (v+w*dist_hulk)
    ve_rad = (v-w*dist_hulk)

    vd = int(vd_rad/max_vel *1000)
    ve = int(ve_rad/max_vel *1000)

    vd_linear = (vd_rad*60/(2*math.pi*raio_hulk))
    ve_linear = (ve_rad*60/(2*math.pi*raio_hulk))

    rospy.loginfo('\nVelocidade da roda direita'  + vd)
    rospy.loginfo('\nVelocidade da roda esquerda' + ve)

    drive.setCommand.goToSpeed(1, vd)
    drive.setCommand.goToSpeed(2, ve)

    time.sleep(0.5)

    drive.setCommand.goToSpeed(1, 0)
    drive.setCommand.goToSpeed(2, 0)



def main():

    drive = gpar_hulk_HDC2450.hdc2450()

    rospy.init_node('gpar_hulk_controll', anonymous=True)
    rospy.Subscriber('/velocidade_hulk', String, recieveData())
    #rospy.Subscriber('/pioneer2dx/cmd_vel', String, recieveData())
    
    rospy.spin()

if __init__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInterruptException:
            pass