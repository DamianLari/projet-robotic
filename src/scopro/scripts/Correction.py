#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import json
import os

class correctionImage:

    def __init__(self):
        #================================
        # Initialisation du noeud ROS et de ses paramètres
        #================================
        rospy.init_node('correct_images', anonymous=True)
       
        self.bridge = CvBridge()
        
        global calibration_file
        current_script_path = os.path.realpath(__file__)
        parent_directory_path = os.path.dirname(current_script_path)
        grandparent_directory_path = os.path.dirname(parent_directory_path)
        file_name = rospy.get_param('~calibration_file')
        calibration_file = os.path.join(grandparent_directory_path, file_name)

        rospy.Subscriber('raw_image_topic', Image, self.image_callback)
        self.corrected_image_pub = rospy.Publisher('corrected_image', Image, queue_size=1)


    def calibration_callback(self):
        #================================
        # Lit les paramètres de calibration depuis le fichier.
        #================================
        try:
            with open(calibration_file) as f:
                data = json.load(f)
            matr = data["mtx"]
            disto = data["dist"]
            return matr , disto
            
        except Exception as e:
            print("Erreur lors de la lecture du fichier de calibration :", e)
    

    def image_callback(self, msg):
        #================================
        # Corrige l'image reçue et la publie sur le sujet ROS.
        # Args:
        #   msg (Image): le message ROS contenant l'image à corriger.
        #================================
        mtx, dist = self.calibration_callback()
        mtx = np.array(mtx)
        dist = np.array(dist)
       
        if mtx is None or dist is None:
            return

        # Conversion de l'image du format ROS au format OpenCV
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8") 

        except CvBridgeError as e:
            print("Erreur lors de la conversion msg_to_cv2:", e)

        h, w = img.shape[:2]
        new_mtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

        # Corriger l'image en utilisant les paramètres de calibration
        corrected_img = cv2.undistort(img, mtx, dist, None, new_mtx)

        # Publier l'image corrigée
        try:
            ros_corrected_img = self.bridge.cv2_to_imgmsg(corrected_img, "bgr8")
           
        except CvBridgeError as e:
            print("Erreur lors de la publication de l'image corrigée:", e)

        print("image corrigée publiée sur: corrected_image")
       
        self.corrected_image_pub.publish(ros_corrected_img)


# Point d'entrée du script

try:
        # Créer une instance de la classe correctionImage
    image_corrector = correctionImage()
        # Attendre les messages ROS
    rospy.spin()
except rospy.ROSInterruptException:
        # Gérer l'exception si le noeud ROS est interrompu
    pass
