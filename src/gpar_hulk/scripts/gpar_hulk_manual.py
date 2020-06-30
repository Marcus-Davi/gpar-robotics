#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import sys,tty,termios, time

class _Getch:
    '''
    Classe usada para pegar apenas um caractere do teclado
    '''
    def __call__(self):
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch



KEYCODE       = [27, 91]
KEYCODE_RIGHT = 67
KEYCODE_LEFT  = 68
KEYCODE_UP    = 65
KEYCODE_DOWN  = 66
KEYCODE_SPACE = 32

k = [None, None, None]
ctr = 1

def read_Keyboard(key):
    '''
    Lê qual seta foi apertada e retorna o valor dela
    ou None para a tecla espaço e -1 para qualquer
    outra tecla
    '''

    if (key[0] == KEYCODE[0] and key[1] == KEYCODE[1]):
        return key[2]
    elif (key[0] == KEYCODE_SPACE):
        return None
    else:
        return -1

def Move():
    '''
    Vê a leitura do teclado e escreve a variavel
    vel (velocidade linear e angular do motor)
    '''
    rospy.init_node('hulk_manual_node', anonymous=True)
    velocity_publisher = rospy.Publisher('/hulk_move', Twist, queue_size=10)
    #velocity_publisher = rospy.Publisher('/pioneer2dx/cmd_vel', Twist, queue_size=10)
    vel = Twist()

    vel.linear.x = 0
    vel.linear.y = 0
    vel.linear.z = 0

    vel.angular.x = 0
    vel.angular.y = 0
    vel.angular.z = 0

    while not rospy.is_shutdown():
        inkey = _Getch()
        while(1):

            for i in range(3):
                k[i] = inkey()
                k[i] = ord(k[i])
                if(k[0] != KEYCODE[0]):
                    break

            arow = read_Keyboard(k)
            #print(arow)
            if arow == None:
                ctr = -1
                break
            #Valor padrao do hulk é 2.0. Simulação recebe 50
            if arow == KEYCODE_UP :
                    print ("up")
                    vel.linear.x  =  50.0
                    vel.angular.z =  0.0

            elif arow == KEYCODE_DOWN:
                    print ("down")
                    vel.linear.x  = -50.0
                    vel.angular.z =  0.0

            elif arow == KEYCODE_RIGHT:
                    print ("right")
                    vel.linear.x  =  0.0
                    vel.angular.z =  50.0

            elif arow == KEYCODE_LEFT:
                    print ("left")
                    vel.linear.x  =  0.0
                    vel.angular.z = -50.0
            else:
                    print ("not an arrow key!")
                    ctr = 1
                    break

            velocity_publisher.publish(vel)
            rospy.loginfo(vel)
            time.sleep(1.5)
            vel.linear.x  =  0.0
            vel.angular.z =  0.0
            velocity_publisher.publish(vel)

        if(ctr == -1):
            break

if __name__ == '__main__':
    try:
        Move()
    except rospy.ROSInterruptException :
        pass
