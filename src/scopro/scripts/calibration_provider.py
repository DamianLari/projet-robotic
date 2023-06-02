import numpy as np
import cv2
import json
import codecs


class CalibrationProvider:
    #==========================================
    # Cette classe permet de fournir la matrice de calibration et les coefficients de distortion
    #==========================================
    def __init__(self, colonnes, lignes):
        #================================
        # Initialisation du CalibrationProvider
        # Args:
        #   colonnes (int): le nombre de colonnes du damier
        #   lignes (int): le nombre de lignes du damier
        #================================
        global cols, rows
        cols = colonnes 
        rows = lignes
       
        global objp,  criteria
        objp = np.zeros((rows * cols, 3), np.float32)
        objp[:,:2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) #* self.taille_case
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

  
    def set_calibration_ros(self,img,pointsobj):
        #================================
        # Trouver les coins du damier dans l'image
        # Args:
        #   img (numpy.array): l'image dans laquelle chercher le damier
        #   pointsobj (list): une liste de points objets
        #
        # Returns:
        #   corners2 (numpy.array): les coins raffinés du damier détecté
        #   pointsobj (list): la liste de points objets mise à jour
        #================================
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, (cols,rows), None)
        corners2 = []
        if ret == True:    
            pointsobj.append(objp)
            # Refine the corners of the detected corners
            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            return corners2, pointsobj
        else: 
            return None,pointsobj

        
   
    def set_calibration_data(self,pointsobj,imgpoints,img_size,calibration_file):
        #================================
        # Effectuer la calibration de la caméra et enregistrer les données dans un fichier.
        # Args:
        #   pointsobj (list): Une liste de points objets.
        #   imgpoints (list): Une liste de points d'image.
        #   img_size (tuple): La taille de l'image (width, height).
        #   calibration_file (str): Le chemin du fichier dans lequel enregistrer les données de calibration.
        #
        # Returns:
        #   mtx (numpy.array): La matrice de calibration de la caméra.
        #   dist (numpy.array): Les coefficients de distortion.
        #================================
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(pointsobj, imgpoints, img_size,None,None)
        
        data = {"mtx":mtx.tolist(),"dist":dist.tolist()}
        json.dump(data, codecs.open(calibration_file, 'w', encoding='utf-8'), 
        separators=(',', ':'), 
        sort_keys=True, 
        indent=4)
        print("Donnée enregistrée dans:", calibration_file)
        
        return mtx , dist