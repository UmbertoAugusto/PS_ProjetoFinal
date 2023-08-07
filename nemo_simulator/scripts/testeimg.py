#!/usr/bin/env python3

'''ESSE CODIGO EH UMA MANEIRA DE PEGAR A IMAGEM, falta saber usa-la'''


import cv2
import sys
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def abrirImagem():
    rospy.init_node('teste_imagem', anonymous=True)
    rospy.Subscriber("camera/image_raw", Image, callback2)
    rospy.wait_for_message("camera/image_raw", Image)

def callback(msg):
    try:
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError:
        pass
    else:
        cv2.imshow("img", img)
        cv2.waitKey(0)

def callback2(msg):
    try:
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError:
        pass
    else:
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        #hsv_green = [60,255,255]
        lower = np.array([50, 100, 100])
        upper = np.array([70, 255, 255])
        mask = cv2.inRange(hsv_img, lower, upper)
        cv2.imshow("img", img)
        cv2.waitKey(0)
        #mostra imagem
        cv2.imshow("img", hsv_img)
        cv2.waitKey(0)
        #mostra só o q eh da cor
        cv2.imshow("hsv_img", mask)
        cv2.waitKey(0)
        #rospy.loginfo(mask)
        

if __name__ == '__main__':
    abrirImagem()