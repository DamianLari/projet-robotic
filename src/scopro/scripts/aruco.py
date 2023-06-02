#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from scopro.msg import ArucoData
from aruco_provider import ArucoProvider
import cv2

class arUco:
    def __init__(self):
        rospy.init_node("aruco_detector")

        global aruco_provider
        # Récupérer les paramètres via rospy
        file_name = rospy.get_param('~calibration_file')
        aruco_size = rospy.get_param('~aruco_size')
        aruco_provider = ArucoProvider(file_name, aruco_size)

        global aruco_pub
        aruco_pub = rospy.Publisher("aruco_markers_data", ArucoData , queue_size=1)
        rospy.Subscriber("corrected_image", Image, self.aruco_detector_callback)

        rospy.spin()
        cv2.destroyAllWindows()

    def aruco_detector_callback(self,image_msg):
        try:
            cv_image = CvBridge().imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))
            return
        ids, rota, transla = aruco_provider.detect_aruco_markers(cv_image)
        print("ids:",ids)
        print("rota: ", rota)
        print("transla", transla)
        if ids is not None:
            aruco_data_msg = self.create_aruco_data_msg(ids, rota, transla, image_msg)
            try:
                aruco_pub.publish(aruco_data_msg)
            except CvBridgeError as e:
                rospy.logerr("CvBridge Error: {}".format(e))
                return
            print("arUco_pose:aruco_detector_callback is done =======")

    def create_aruco_data_msg(self, ids, rota, transla, image_msg):
        for i in range(len(ids)):
                twist_msg = Twist()
                twist_msg.linear.x = transla[i][0]
                twist_msg.linear.y = transla[i][1]
                twist_msg.linear.z = transla[i][2]
                twist_msg.angular.x = rota[i][0]
                twist_msg.angular.y = rota[i][1]
                twist_msg.angular.z = rota[i][2]
                print("twist_msg:",twist_msg)
                aruco_data_msg = ArucoData()
                aruco_data_msg.image = image_msg
                aruco_data_msg.ids = ids[i][0]
                aruco_data_msg.poses = twist_msg
                print("aruco_data_msg.ids:",aruco_data_msg.ids)
                print("aruco_data_msg.poses:",aruco_data_msg.poses)
                
                return aruco_data_msg
                
               
        pass

if __name__ == "__main__":
    try:
        aaruco_detector = arUco()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
