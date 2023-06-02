#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2

#import cv2.aruco as aruco
from cv2 import aruco
import numpy as np
import scipy.spatial.transform as R
from scopro.msg import ArucoData
from std_msgs.msg import String
import json
import os
import time

class arUco:
    def __init__(self):
        #================================
        # Initialise le noeud ROS et charge les données de calibration.
        #================================
        rospy.init_node("aruco_detector")

        global calibration_file
        current_script_path = os.path.realpath(__file__)
        parent_directory_path = os.path.dirname(current_script_path)
        grandparent_directory_path = os.path.dirname(parent_directory_path)
        file_name = rospy.get_param('~calibration_file')
        calibration_file = os.path.join(grandparent_directory_path, file_name)

        # Charger les données de calibration
        global camera_matrix, dist_coeffs
        camera_matrix , dist_coeffs = self.load_calib_data()
        camera_matrix = np.array(camera_matrix)
        dist_coeffs = np.array(dist_coeffs)

        global aruco_dict, aruco_size, num_img
        aruco_dict_name = rospy.get_param('~aruco_dictionary')
        #aruco_dict = getattr(aruco, aruco_dict_name)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
        aruco_size = rospy.get_param('~aruco_size')
        num_img = 1

        global aruco_pub ,aruco_img_pub, aruco_ids_pub, aruco_twist
        aruco_pub = rospy.Publisher("aruco_markers_data", ArucoData , queue_size=1)
        rospy.Subscriber("corrected_image", Image, self.aruco_detector_callback)

        rospy.spin()
        cv2.destroyAllWindows()


    def load_calib_data(self):
        #================================
        # Charge les données de calibration depuis le fichier spécifié.
        #================================
        try:
            with open(calibration_file) as f:
                data = json.load(f)
            matr = data["mtx"]
            disto = data["dist"]
          
        except Exception as e:
            print("Erreur lors de la lecture du fichier de calibration :", e)
        return matr , disto
    
    def detect_aruco_markers(self,image, camera_matrix, dist_coeffs,output_folder="output"):
        #================================
        # Détecte les marqueurs ArUco dans l'image.
        # Args:
        #   image (ndarray): l'image à analyser.
        #   camera_matrix (ndarray): la matrice de calibration de la caméra.
        #   dist_coeffs (ndarray): les coefficients de distorsion de la caméra.
        #================================
        aruco_params = aruco.DetectorParameters_create()
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(image, aruco_dict, parameters=aruco_params)
       
        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, aruco_size, camera_matrix, dist_coeffs)
            image_with_markers = aruco.drawDetectedMarkers(image.copy(), corners, ids)
            for i in range(len(rvecs)):
                image_with_markers = aruco.drawAxis(image_with_markers, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1)


            try:
                # Crée le dossier de sortie s'il n'existe pas.
                os.makedirs(output_folder, exist_ok=True)

                # Crée un nom de fichier unique basé sur l'horodatage.
                timestamp = time.strftime("%Y%m%d-%H%M%S")
                output_filename = os.path.join(output_folder, f"aruco_{timestamp}.jpeg")
                print("Image avec markers dessinés:",output_filename)
                # Sauvegarde l'image.
                success = cv2.imwrite(output_filename, image_with_markers)
                print(f"Image sauvegardée avec succès: {success}")
                if not cv2.imwrite(output_filename, image_with_markers):  # Vérifie le retour de cv2.imwrite
                    raise Exception("Could not write image")  # Si cv2.imwrite renvoie False, lève une exception
                
                return corners, ids, rvecs, tvecs, output_filename
            except Exception as e:
                print(f"Erreur lors de la sauvegarde de l'image: {e}")
                return corners, ids, rvecs, tvecs, None

            """
            print("rvecs:", rvecs)
            print("tvecs:", tvecs)
            """
            return corners, ids, rvecs, tvecs
        
        else:
            return None, None, None, None
   
    def aruco_detector_callback(self,image_msg):
        #================================
        # Traite l'image reçue pour détecter les marqueurs ArUco et publie les données sur le sujet ROS.
        # Args:
        #   image_msg (Image): le message ROS contenant l'image à analyser.
        #================================
        try:
            cv_image = CvBridge().imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))
            return
        aruco_data_msg = ArucoData()
        # Détecter les marqueurs ArUco
        corners, ids, rvecs, tvecs, output_filename = self.detect_aruco_markers(cv_image, camera_matrix, dist_coeffs,"/root/scopro_ws/src/scopro/draw_jaguaruco")
        #print("Image avec markers dessinés:",output_filename)
        try:
            print("nombre de marqueurs ArUco : ", len(ids))
        except:
            print("aucun marqueur ArUco trouvé")
        if ids is not None:
            # Convertir les rotations (rvecs) en matrices de rotation
            rotation_matrices = [cv2.Rodrigues(rvec)[0] for rvec in rvecs]

            # Créer des matrices homogènes 4x4
            homogeneous_matrices = []
            for i in range(len(rotation_matrices)):
                H = np.identity(4)
                H[0:3, 0:3] = rotation_matrices[i]
                H[0:3, 3] = tvecs[i].ravel()
                homogeneous_matrices.append(H)

            # Multiplier à gauche par la matrice de transformation
            transformation_matrix = np.array([
                [0, -1, 0, 0],
                [0, 0, -1, 0],
                [1, 0, 0, 0],
                [0, 0, 0, 1]
            ])
            
            new_homogeneous_matrices = []
            for H in homogeneous_matrices:
                new_H = np.dot(transformation_matrix, H)
                new_homogeneous_matrices.append(new_H)

            # Obtenir les angles d'Euler et les translations à partir des nouvelles matrices homogènes
            rota = []
            transla = []
            for H in new_homogeneous_matrices:
                rot = R.Rotation.from_matrix(H[0:3, 0:3]).as_euler("ZYX", degrees=False)
                rota.append(np.flip(rot))
                transla.append(H[0:3, 3])

            # Publier les données des marqueurs ArUco détectés

            for i in range(len(ids)):
                twist_msg = Twist()
                twist_msg.linear.x = transla[i][0]
                twist_msg.linear.y = transla[i][1]
                twist_msg.linear.z = transla[i][2]
                twist_msg.angular.x = rota[i][0]
                twist_msg.angular.y = rota[i][1]
                twist_msg.angular.z = rota[i][2]
                #aruco_data_msg = ArucoData()
                aruco_data_msg.image = image_msg
                aruco_data_msg.ids = ids[i][0]
                aruco_data_msg.poses = twist_msg
                print("aruco_data_msg.ids:",aruco_data_msg.ids)
                print("aruco_data_msg.poses:",aruco_data_msg.poses)
            aruco_pub.publish(aruco_data_msg)
           

        # Publier l'image non corrigée avec les marqueurs ArUco détectés
        try:
            aruco_pub.publish(aruco_data_msg)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))
            return
       
        print("arUco_pose:aruco_detector_callback is done =======")
       

   

if __name__ == "__main__":
    #================================
    # Point d'entrée du script. Crée une instance de la classe arUco et attend les messages ROS.
    #================================
    try:
        aaruco_detector = arUco()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
