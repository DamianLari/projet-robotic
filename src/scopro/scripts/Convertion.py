#!/usr/bin/env python3
import os
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String



class conversionImage:
    #==========================================
    # Cette classe permet de convertir des images en messages ROS
    #==========================================
    def __init__(self):
        #================================
        # Initialisation du noeud ROS
        #================================
        rospy.init_node('image_publisher', anonymous=True)
       
        # Création d'un éditeur pour le sujet 'raw_image_topic'
        self.image_pub = rospy.Publisher('raw_image_topic', Image, queue_size=10)
        
        global isDone
        isDone = rospy.Publisher('conv_done',String,queue_size=10)

        # Définition de la fréquence de publication
        self.rate = rospy.Rate(4)

        # Création d'une instance de CvBridge pour convertir les images OpenCV en messages ROS
        self.bridge = CvBridge()

        # Spécifier le chemin du dossier contenant les images
        global dataset_directory, script_directory, image_names_without_extension, sorted_image_names
        # Trouver le chemin du répertoire du script en cours d'exécution
        script_directory = os.path.dirname(os.path.abspath(__file__))
        folder = rospy.get_param('~folder')
        dataset_directory = os.path.join(script_directory, '..', folder)
        
        # Récupérer la liste des fichiers du dossier
        file_list = os.listdir(dataset_directory)

        # Filtrer la liste pour ne conserver que les fichiers image (par exemple, les fichiers .jpg)
        image_list = [file for file in file_list if file.endswith(".jpg")]
        sorted_image_names = sorted(image_list)
        image_names_without_extension = [os.path.splitext(image_name)[0] for image_name in sorted_image_names]
        self.nb_rows_images = len(sorted_image_names)

        todo = rospy.get_param('~todo')
        if todo == "calib":
            nb_input_images = input("Combien d'images pour la calibration? <value> or 'all'")
            if nb_input_images.isnumeric():
                self.nb_rows_images = int(nb_input_images)
                print("nombre d'images voulu:",self.nb_rows_images)

        elif todo == "aruco":
            nb_input_images = input("Lire en boucle le même dataset? y or n")
            if nb_input_images == "y":
                while True:
                    self.image_publisher()

        
        

    def image_publisher(self):
        #================================
        # Lit, convertit et publie des images sur le sujet ROS.
        #================================
        image_envoyée = 1
        for img_name in sorted_image_names[0:self.nb_rows_images]:
            # Si le fichier est une image au format JPG
                
            if img_name.endswith("jpg"):
                # Créer le chemin complet de l'image
                img_path = os.path.join(dataset_directory, img_name)
                print ("Nom de l'image:", img_path)
                # Lire l'image en utilisant OpenCV
                img = cv2.imread(img_path, 1)
            
                try:
                    ros_img = self.bridge.cv2_to_imgmsg(img,"bgr8") #bgr8
                    #ros_img.header.stamp = rospy.Time.now()
                except CvBridgeError as e:
                    print("Erreur lors de la conversion cv2_to_msg:", e)

                # Publier l'image sur le sujet ROS
                self.image_pub.publish(ros_img)
                  
                # Attendre avant de publier la prochaine image (en fonction de la fréquence définie)
                self.rate.sleep()

                print("image",image_envoyée,"publiée")
                image_envoyée += 1
        print("Convertion: image_publisher is done =======")
           

# Point d'entrée du script

try:
        # Créer une instance de la classe conversionImage
    image_converter = conversionImage()
        # Appeler la méthode image_publisher
    #image_converter.image_publisher()
except rospy.ROSInterruptException:
        # Gérer l'exception si le noeud ROS est interrompu
    pass

