#!/usr/bin/env python3
import socket
import json

import rospy
from geometry_msgs.msg import Pose
from scopro.msg import MocapData

rospy.init_node('mocap_node')
pub = rospy.Publisher('mocap_topic', MocapData, queue_size=10)

mocap_data = MocapData()

# Créer un socket
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    # Se connecter au serveur
    s.connect(('host.docker.internal', 11311))
    while not rospy.is_shutdown():
        # Recevoir des données
        data = s.recv(1024)
        if not data:
            break
        # Convertir les données en dictionnaire
        try:
            obj_dict = json.loads(data.decode())
        except json.JSONDecodeError:
            print("Les données reçues ne sont pas au format JSON valide", data)
            continue
        
        # Vider les listes avant de les remplir à nouveau
        mocap_data.poses = []
        mocap_data.ids = []

        for id, pose_objets in obj_dict.items():
            try:
                # Convertir chaque élément de la liste en flottant (excepté le premier élément qui est un entier)
                pose_objets = [float(x) if i != 0 else int(x) for i, x in enumerate(pose_objets)]
            except ValueError:
                print("Une entrée non conforme a été ignorée", id, pose_objets)
                continue
            
            # Créer et ajouter la pose
            pose = Pose()
            pose.position.x = pose_objets[2]
            pose.position.y = pose_objets[3]
            pose.position.z = pose_objets[4]
            pose.orientation.x = pose_objets[5]
            pose.orientation.y = pose_objets[6]
            pose.orientation.z = pose_objets[7]
            pose.orientation.w = pose_objets[8]
            mocap_data.poses.append(pose)

            # Ajouter l'ID
            mocap_data.ids.append(int(id))
            
        # Mettre à jour le timestamp avec le dernier timestamp reçu
        mocap_data.header.stamp = rospy.Time.from_sec(pose_objets[0])

        print("=======================================")
        print (mocap_data)
        print("=======================================")

        # Publier le message MocapData
        pub.publish(mocap_data)
