#!/bin/env python3
import roslib
import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard
from pynput.keyboard import Key, Controller

# PET: ler teclas e converter para mensagem de velocidade pro robô

# formato mensagem : TWIST 
# TWIST = {velocidade_linear, velocidade_angular}
# velocidade_linear = {x, y,z}
# velocidade_angular = {x,y,z}
#apenas linear_x e angular_z são usadas

if __name__=='__main__':
    rospy.init_node("nanook_mover")
    publicador = rospy.Publisher("/nanook_move",Twist,queue_size=5)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():

        rospy.loginfo("Loop!")
        mensagem = Twist()


        with keyboard.Events() as events:
            event = events.get(1e6)
            # testar de ficou vazio
            if event.key == keyboard.KeyCode.from_char('w'):
                mensagem.linear.x = 0.1 # m/s
                mensagem.angular.z = 0.0 # rad/s
                publicador.publish(mensagem)

            elif event.key == keyboard.KeyCode.from_char('s'):
                mensagem.linear.x = -0.1 # m/s
                mensagem.angular.z = 0.0 # rad/s
                publicador.publish(mensagem)

            elif event.key == keyboard.KeyCode.from_char('a'):
                mensagem.linear.x =  0.0 # m/s
                mensagem.angular.z = 0.5 # rad/s
                publicador.publish(mensagem)
            elif event.key == keyboard.KeyCode.from_char('d'):
                mensagem.linear.x =  0.0 # m/s
                mensagem.angular.z = -0.5 # rad/s
                publicador.publish(mensagem)



         
        #publicador.publish(mensagem)
        # rate.sleep()



