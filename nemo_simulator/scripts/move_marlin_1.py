#!/usr/bin/env python3
import cv2
import rospy
import tf2_ros
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PointStamped
from gazebo_msgs.msg import ModelStates
from cv_bridge import CvBridge, CvBridgeError


class testarMovimento():
    def __init__(self):
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera/image_raw", Image, self.callback_image)
        self.vel_nemo_sub = rospy.Subscriber("/vel_nemo", Twist, self.callback_vel_nemo)
        self.vendo_nemo = False
        self.center_x = 0
        self.image_width = 640
        self.distancia = 0


    def callback_image(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            #Converte a imagem BGR para o espaço de cores HSV
            hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            # Define os valores mínimos e máximos para o filtro
            lower = np.array([50, 100, 100])
            upper = np.array([70, 255, 255])

            mask = cv2.inRange(hsv_img, lower, upper)
            
            #Encontra o maior contorno na máscara.
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                #Encontra o contorno de maior área entre os contornos
                largest_contour = max(contours, key=cv2.contourArea)

                moments = cv2.moments(largest_contour)
                if moments["m00"] != 0:
                    #Calcula o centro da área branca com base nos momentos do contorno
                    self.center_x = int(moments["m10"] / moments["m00"])
                    self.vendo_nemo = True
                    
                else:
                    #Define o centro como 0 se não tiver área branca
                    self.center_x = 0

            else:
                # Define o centro x como 0 se nenhum contorno for detectado
                self.center_x = 0

            #cv2.imshow("mask", mask)
            #cv2.waitKey(1)

        except CvBridgeError:
            pass


    def callback_vel_nemo(self, msg):
        # Definição da função de callback para processar os dados do sonar.

        if not self.vendo_nemo:
            self.procurar()

        else:
            vel_lin_x = msg.linear.x
            vel_lin_y = msg.linear.y
            # Obtém as componentes da velocidade do nemo.

            modulo_velocidade_linear = (vel_lin_x ** 2 + vel_lin_y ** 2) ** 0.5
            # Calcula o módulo da velocidade utilizando o teorema de Pitágoras.
        
            self.seguir(modulo_velocidade_linear)
            # Chama a função "seguir" para controlar o movimento do robô.

    def procurar(self):
        msg_enviar = Twist()

        msg_enviar.linear.x = 0
        msg_enviar.linear.y = 1
        msg_enviar.linear.z = 0
        msg_enviar.angular.x = 0
        msg_enviar.angular.y = 0
        msg_enviar.angular.z = 1

        self.pub.publish(msg_enviar)


    def seguir(self, modulo_velocidade_linear):
        msg_enviar = Twist()

        #Velocidades lineares
        msg_enviar.linear.x = 0 #componente_x
        msg_enviar.linear.y = modulo_velocidade_linear
        msg_enviar.linear.z = 0

        #Rotaciona o robô se o centro da área branca não estiver no meio da imagem.
        #Calcula a velocidade angular com base na diferença entre o centro da área branca e o centro da imagem.
        #A velocidade angular é limitada para não exceder um valor máximo.
        angular_speed = 1
        angular_factor = 0.1
        angular_z = -(self.center_x - self.image_width) * angular_factor
        
        if abs(angular_z) < angular_speed:
            msg_enviar.angular.z = angular_z

        else:
            msg_enviar.angular.z = angular_z / abs(angular_z) * angular_speed

        self.pub.publish(msg_enviar)
        

if __name__ == '__main__':
    rospy.init_node("teste_movimento", anonymous=True)
    teste1 = testarMovimento()
    rospy.spin()