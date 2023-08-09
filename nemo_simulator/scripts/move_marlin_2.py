#!/usr/bin/env python3
import rospy
import tf2_ros
import cv2
import time
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge, CvBridgeError



'''Esse codigo eh apenas um teste. Podemos usar ele mesmo, alterando os nomes.'''

class testarMovimento():
    def __init__(self):
        self.__vendo_nemo_direita = False
        self.__vendo_nemo_esquerda = False
        self.__vendo_nemo_centro = False
        self.__modulo_vel_nemo = 0
        self.__modulo_distancia = 0
        self.__esquerda = []
        self.__meio = []
        self.__direita = []
        self.bridge = CvBridge()

        #self.distancia_relativa()
        self.vel_nemo()
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        #chama metodo para achar modulo da velocidade do nemo:    

    def get_vendo_nemo(self):
        return self.__vendo_nemo_direita, self.__vendo_nemo_esquerda, self.__vendo_nemo_centro

    def reset_status_esquerda(self):
        self.__vendo_nemo_esquerda = False
    
    def reset_status_direita(self):
        self.__vendo_nemo_direita = False
    
    def reset_status_centro(self):
        self.__vendo_nemo_centro = False

    def vel_nemo(self):
        rospy.Subscriber("/sonar_data", Twist, self.callback_vel_nemo)

    def callback_vel_nemo(self,msg):
        #pega componentes da velocidade do nemo
        vel_nemo_x = msg.linear.x
        vel_nemo_y = msg.linear.y
        vel_nemo_z = msg.linear.z
        #calcula modulo da velocidade do nemo para que o marlin ande na mesma velocidade.
        self.__modulo_vel_nemo = (vel_nemo_x**2+vel_nemo_y**2)**(1/2)

    '''def distancia_relativa(self):
        rospy.Subscriber("/vel_nemo", Twist, self.callback_distancia)
    
    def callback_distancia(self,msg):
        dist_x = msg.point.x
        dist_y = msg.point.y
        dist_z = msg.point.z
        self.__modulo_distancia = (msg.point.x**2+msg.point.y**2)**(1/2)'''

    def pegar_imagem(self):
        #r = rospy.Rate(10)

        teste1.vel_nemo()
        #teste1.distancia_relativa()
        #rospy.Subscriber("camera/image_raw", Image, self.callback_img)
        rospy.Subscriber("camera/image_raw", Image, self.process_img)


        #return self.__esquerda, self.__meio, self.__direita
        #r.sleep()

    def set_msg_img(self, img):
        msg_img = img
        self.process_img(msg_img)

    '''def process_img(self, msg_img):
        try:
            #tranforma o tipo da msg para que seja possivel trabalhar com ela.
            bridge = CvBridge()
            img = bridge.imgmsg_to_cv2(msg_img, "bgr8")
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
        

        self.__esquerda = esquerda
        self.__meio = meio
        self.__direita = direita'''

    def process_img(self, msg_img):
        try:
            img = self.bridge.imgmsg_to_cv2(msg_img, "bgr8")
            
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

                else:
                    #Define o centro como 0 se não tiver área branca
                    self.center_x = -1

            else:
                # Define o centro x como 0 se nenhum contorno for detectado
                self.center_x = -1

        except CvBridgeError:
            pass

        else:
            if 0 < self.center_x < 427:
                self.__vendo_nemo_esquerda = True
            elif 427 < self.center_x < 855:
                self.__vendo_nemo_centro = True
            elif 855 < self.center_x:
                self.__vendo_nemo_direita = True

        
    '''def callback_img(self, esquerda, meio, direita):
            

            self.__vendo_nemo_esquerda = False
            for linha in esquerda:

                if 255 in linha:
                    self.__vendo_nemo_esquerda = True
                    print("Objeto identificado na esquerda")
                    return 1

            self.__vendo_nemo_direita = False
            for linha in direita:
    
                if 255 in linha:
                    self.__vendo_nemo_direita = True
                    print("Objeto identificado na direita")
                    return 1

            self.__vendo_nemo_centro = False
            for linha in meio:
            
                if 255 in linha:
                    self.__vendo_nemo_centro= True
                    print("Objeto identificado no centro")
                    return 1
                
            return 0'''

    def girar_esquerda(self):
            msg_enviar = Twist()
            #pro carrinho andar pra frente, ele anda na direção y apenas
            msg_enviar.linear.x = 0
            msg_enviar.linear.y = .5#self.modulo_vel_nemo
            msg_enviar.linear.z = 0
            #girar no eixo z apenas
            msg_enviar.angular.x = 0
            msg_enviar.angular.y = 0
            msg_enviar.angular.z = 1
            
            self.pub.publish(msg_enviar)


    def girar_direita(self):
            msg_enviar = Twist()
            #pro carrinho andar pra frente, ele anda na direção y apenas
            msg_enviar.linear.x = 0
            msg_enviar.linear.y = .5#self.modulo_vel_nemo
            msg_enviar.linear.z = 0
            #girar no eixo z apenas
            msg_enviar.angular.x = 0
            msg_enviar.angular.y = 0
            msg_enviar.angular.z = -1
            
            self.pub.publish(msg_enviar)


    def seguir(self):
            msg_enviar = Twist()
            #pro carrinho andar pra frente, ele anda na direção y apenas
            msg_enviar.linear.x = 0
            msg_enviar.linear.y = 1#self.__modulo_vel_nemo*self.__modulo_distancia
            msg_enviar.linear.z = 0
            #girar no eixo z apenas
            msg_enviar.angular.x = 0
            msg_enviar.angular.y = 0
            msg_enviar.angular.z = 0
            print("seguindo com velocidade 1")

            self.pub.publish(msg_enviar)

    def procurar_nemo(self):
        #if self.__vendo_nemo_direita == False and self.__vendo_nemo_esquerda == False and self.__vendo_nemo_centro == False:
            msg_enviar = Twist()
            #pro carrinho andar pra frente, ele anda na direção y apenas
            msg_enviar.linear.x = 0
            msg_enviar.linear.y = 0
            msg_enviar.linear.z = 0
            
            #girar no eixo z apenas
            msg_enviar.angular.x = 0
            msg_enviar.angular.y = 0
            msg_enviar.angular.z = 2

            self.pub.publish(msg_enviar)
            return 1

if __name__ == '__main__':
    rospy.init_node("teste_movimento", anonymous=True)

    teste1 = testarMovimento()
    while not rospy.is_shutdown():
        
        print("Pegando Imagem")

        teste1.pegar_imagem()
        '''esquerda, meio, direita = teste1.pegar_imagem()
        if teste1.callback_img(esquerda, meio, direita) == 1:
            if teste1.girar_esquerda() == 1:
                print ("girando para a esquerda")  
                teste1.reset_status_esquerda()      
                continue

            if teste1.girar_direita() == 1:
                print("girando para a direita")
                teste1.reset_status_direita()      
                continue

            print("seguindo")
            teste1.seguir()
            teste1.reset_status_centro()  '''
        vendo_nemo_direita, vendo_nemo_esquerda, vendo_nemo_centro = teste1.get_vendo_nemo()
        if vendo_nemo_esquerda:
            teste1.girar_esquerda()
            print ("girando para a esquerda")
            teste1.reset_status_esquerda()
        elif vendo_nemo_direita:
            teste1.girar_direita()
            print("girando para a direita")
            teste1.reset_status_direita()
        elif vendo_nemo_centro:
            teste1.seguir()
            print("seguindo")
            teste1.reset_status_centro()
        else:
            #if teste1.procurar_nemo() == 1:
            teste1.procurar_nemo()
            print ("procurando")

        time.sleep(1)
        
        #esse tempo de espera deve ser parametrizado