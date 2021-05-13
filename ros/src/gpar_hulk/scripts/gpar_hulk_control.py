#!/usr/bin/env python3

import sys
import rospy
from geometry_msgs.msg import Twist
from gpar_hulk_HDC2450 import *
import math
import time

dist_hulk = 0.4  #averiguar
raio_hulk = 0.07  #averiguar
max_vel = 300 #rpm
max_vel_ang = 2 * max_vel / dist_hulk
drive = hdc2450()

def receiveData(vel):

    v = vel.linear.x
    w = vel.angular.z

    vd_lin = (v+w*0.5*dist_hulk) #linear
    ve_lin = (v-w*0.5*dist_hulk) #linear

    '''
    formulas vistas no curso de robótica

    vd = (2*v + w*dist_hulk) / 2*raio_hulk
    ve = (2*v - w*dist_hulk) / 2*raio_hulk
    
    '''
    vd_rad = vd_lin/raio_hulk # m/s -> rad/s
    ve_rad = ve_lin/raio_hulk
   

    vd_rpm = int(vd_rad*60/(2*math.pi)) # rad/s -> rpm
    ve_rpm = int(ve_rad*60/(2*math.pi))

    rospy.loginfo('\nVelocidade da roda direita: '  + str(vd_rpm))
    rospy.loginfo('\nVelocidade da roda esquerda: ' + str(ve_rpm))

    drive.setCommand.goToSpeed(vd_rpm, ve_rpm)

def main():

    rospy.init_node('gpar_hulk_controll', anonymous=True)
    rospy.Subscriber('/velocidade_hulk', Twist, receiveData)
    #rospy.Subscriber('/pioneer2dx/cmd_vel', String, recieveData())
    rospy.loginfo('\Esperando receber Twist !')
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
            pass