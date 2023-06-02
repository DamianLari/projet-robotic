import cv2
from cv2 import aruco
import numpy as np
import scipy.spatial.transform as R
import json

class ArucoProvider:
    def __init__(self, calibration_file, aruco_size):
        self.camera_matrix, self.dist_coeffs = self.load_calib_data(calibration_file)
        self.camera_matrix = np.array(self.camera_matrix)
        self.dist_coeffs = np.array(self.dist_coeffs)
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
        self.aruco_size = aruco_size

    def load_calib_data(self, calibration_file):
        try:
            with open(calibration_file) as f:
                data = json.load(f)
            matr = data["mtx"]
            disto = data["dist"]
        except Exception as e:
            print("Erreur lors de la lecture du fichier de calibration :", e)
        return matr , disto

    def detect_aruco_markers(self, image):
        aruco_params = aruco.DetectorParameters_create()
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(image, self.aruco_dict, parameters=aruco_params)
        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.aruco_size, self.camera_matrix, self.dist_coeffs)
            rota, transla = self.calculate_positions(ids, rvecs, tvecs)
            """
            print("rvecs: ", rvecs)
            print("tvecs: ", tvecs)
            print("rota: ", rota)
            print("transla", transla)
            """
            return ids, rota, transla
        else:
            return None, None, None
    
    def calculate_positions(self, ids, rvecs, tvecs):
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

            return rota, transla

        return None, None
