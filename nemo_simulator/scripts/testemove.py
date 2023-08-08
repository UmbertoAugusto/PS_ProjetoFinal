#!/usr/bin/env python3

import rospy
import tf2_ros
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
import numpy as np

'''Esse codigo serviu de teste, por isso os nomes estao estranhos. Podemos usar ele mesmo, alterando os nomes.'''


class testarMovimento():
    def teste(self):
        #define um publisher que vai publicar no topico "/cmd_vel" uma mensagem do tipo Twist.
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        #define uma variavel do tipo Twist
        msg_enviar = Twist()
        #define as componentes das velocidades linear e angular.
        #linear, nos eixos x, y e z:
        msg_enviar.linear.x = 0 
        msg_enviar.linear.y = 2 
        msg_enviar.linear.z = 0 
        #angular, nos eixos x, y e z:
        msg_enviar.angular.x = 0 
        msg_enviar.angular.y = 0 
        msg_enviar.angular.z = 0 
        #publica a mensagem.
        pub.publish(msg_enviar)

if __name__ == '__main__':
    #inicia um nodo.
    rospy.init_node("teste_movimento", anonymous=True)
    #cria um obeto da classe testarMovimento.
    teste1 = testarMovimento()
    
    #laco de repeticao para publicar constantemente a velocidade do carrinho.
    #acho que dentro desse laco de repeticao a gente deve calcular a velocidade do carrinho.
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        teste1.teste()
        r.sleep()