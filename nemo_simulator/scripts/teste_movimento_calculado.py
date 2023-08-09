#!/usr/bin/env python3

import rospy
import tf2_ros
import cv2
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

'''Esse codigo eh apenas um teste. Podemos usar ele mesmo, alterando os nomes.'''


class testarMovimento():
    def __init__(self):
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        #chama metodo para achar modulo da velocidade do nemo:    

    def vel_nemo(self):
        rospy.Subscriber("/sonar_data", Twist, self.callback_vel_nemo)

    def callback_vel_nemo(self,msg):
        #pega componentes da velocidade do nemo
        vel_nemo_x = msg.linear.x
        vel_nemo_y = msg.linear.y
        vel_nemo_z = msg.linear.z
        #calcula modulo da velocidade do nemo para que o marlin ande na mesma velocidade.
        self.modulo_vel_nemo = (vel_nemo_x**2+vel_nemo_y**2)**(1/2)

    def distancia_relativa(self):
        rospy.Subscriber("/vel_nemo", Twist, self.callback_distancia)
    
    def callback_distancia(self,msg):
        dist_x = msg.point.x
        dist_y = msg.point.y
        dist_z = msg.point.z
        self.modulo_distancia = (msg.point.x**2+msg.point.y**2)**(1/2)

    def pegar_imagem(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            teste1.vel_nemo()
            teste1.distancia_relativa()
            rospy.Subscriber("camera/image_raw", Image, self.callback_img)
            r.sleep()

    def callback_img(self,msg):
            try:
                #tranforma o tipo da msg para que seja possivel trabalhar com ela.
                bridge = CvBridge()
                img = bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError:
                pass
            else:
                #transforma a imagem para HSV.
                hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                #hsv_green = [60,255,255]
                lower = np.array([50, 100, 100])
                upper = np.array([70, 255, 255])
                mask = cv2.inRange(hsv_img, lower, upper)
            
                esquerda = mask[:,:427]
                meio = mask[:,427:855]
                direita = mask[:,855:]

                vendo_nemo = False
                for linha in esquerda:
                    if 255 in linha:
                        vendo_nemo = True
                        self.girar_esquerda()
                        break
                if vendo_nemo == False:
                    for linha in direita:
                        if 255 in linha:
                            vendo_nemo = True
                            self.girar_direita()
                            break
                if vendo_nemo == False:
                    for linha in meio:
                        if 255 in linha:
                            vendo_nemo = True
                            self.seguir()
                            break
                if vendo_nemo == False:
                    self.girar_esquerda()

    def girar_esquerda(self):
        msg_enviar = Twist()
        #pro carrinho andar pra frente, ele anda na direção y apenas
        msg_enviar.linear.x = 0
        msg_enviar.linear.y = 2#self.modulo_vel_nemo
        msg_enviar.linear.z = 0
        #girar no eixo z apenas
        msg_enviar.angular.x = 0
        msg_enviar.angular.y = 0
        msg_enviar.angular.z = 4

        self.pub.publish(msg_enviar)

    def girar_direita(self):
        msg_enviar = Twist()
        #pro carrinho andar pra frente, ele anda na direção y apenas
        msg_enviar.linear.x = 0
        msg_enviar.linear.y = 2#self.modulo_vel_nemo
        msg_enviar.linear.z = 0
        #girar no eixo z apenas
        msg_enviar.angular.x = 0
        msg_enviar.angular.y = 0
        msg_enviar.angular.z = -4

        self.pub.publish(msg_enviar)

    def seguir(self):
        msg_enviar = Twist()
        #pro carrinho andar pra frente, ele anda na direção y apenas
        msg_enviar.linear.x = 0
        msg_enviar.linear.y = 2#self.modulo_vel_nemo*self.modulo_distancia
        msg_enviar.linear.z = 0
        #girar no eixo z apenas
        msg_enviar.angular.x = 0
        msg_enviar.angular.y = 0
        msg_enviar.angular.z = 0

        self.pub.publish(msg_enviar)


if __name__ == '__main__':
    rospy.init_node("teste_movimento", anonymous=True)
    
    teste1 = testarMovimento()

    teste1.pegar_imagem()