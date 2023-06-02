#!/usr/bin/env python3
import rospy
from cv2 import aruco
import os
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from calibration_provider import CalibrationProvider
import threading
import time




class CameraCalibration:
    #==========================================
    # Cette classe permet de faire la calibration de la caméra à l'aide de ROS
    #==========================================
    def __init__(self):
        #================================
        # Initialisation du noeud ROS et des paramètres de calibration
        #================================
        rospy.init_node('camera_calibration', anonymous=True)
       
        global number_columns, number_rows, calibration_file
        number_columns = rospy.get_param('~number_cols')
        number_rows = rospy.get_param('~number_rows')
        calibration_file = rospy.get_param('~calibration_file')
        
        global calibration_path
        current_script_path = os.path.realpath(__file__)
        parent_directory_path = os.path.dirname(current_script_path)
        grandparent_directory_path = os.path.dirname(parent_directory_path)
        file_name = rospy.get_param('~calibration_file')
        calibration_path = os.path.join(grandparent_directory_path, file_name)


        global point_image
        point_image = []
        self.point_objet = []
    
        self.bridge = CvBridge()
        rospy.Subscriber('raw_image_topic', Image,self.image_callback)

        # Dernière réception d'une image
        self.last_received_time = None

        # Thread pour vérifier le timeout
        self.timeout_thread = threading.Thread(target=self.check_timeout)
        self.timeout_thread.start()

   
    def image_callback(self,msg):
        #================================
        # Callback pour le traitement des images
        # Args:
        #   msg (ros message): le message reçu du topic d'image
        #================================
        self.last_received_time = rospy.get_time()
       
        try:
            global img
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print("Erreur lors de la conversion msg_to_cv2:", e)
            return
       
        self.calibration_provider = CalibrationProvider(number_columns, number_rows)
        corners, self.point_objet = self.calibration_provider.set_calibration_ros(img,self.point_objet)
        global point_image
        if corners is not None: 
            point_image.append(corners) 
        print("image traitées:",len(point_image))
        print("================================================")

     

    def check_timeout(self):
        #================================
        # Vérifie si un timeout est survenu et effectue la calibration finale si c'est le cas
        #================================
        timeout = 2.5  # Timeout en secondes

        while not rospy.is_shutdown():
            if self.last_received_time and (rospy.get_time() - self.last_received_time) > timeout:
                rospy.loginfo("Timeout détecté, calcul final de mtx et dist")
                global img , point_image
                img_size = (img.shape[1], img.shape[0])
                if len(self.point_objet) > 0 and len(point_image) > 0:
                    print("début du calcul de mtx et dist")
                    start_time = time.time()
                    mtx, dist = self.calibration_provider.set_calibration_data(self.point_objet, point_image, img_size, calibration_path)
                    elapsed_time = time.time() - start_time
                    print("Temps de calcul :{:.2f} secondes".format(elapsed_time))
                    print("mtx:", mtx)
                    print("dist:", dist)

                else:
                    print("Pas assez de données pour effectuer la calibration.")
                break
            rospy.sleep(1)

  

if __name__ == '__main__':
    #================================
    # Instancier la classe CameraCalibration et lancer le noeud ROS
    #================================
    try:
        camera_calibrator = CameraCalibration()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
