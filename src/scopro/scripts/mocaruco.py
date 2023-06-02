#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose, Vector3, PoseStamped
from scopro.msg import MocapData, ArucoData
from tf2_ros import TransformStamped, Buffer, TransformListener
import tf_conversions

class ArucoInWorldFrame:
    def __init__(self):
        self.aruco_pose = None
        self.camera_pose = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        self.aruco_sub = rospy.Subscriber('/aruco_markers_data', ArucoData, self.aruco_callback)
        self.camera_sub = rospy.Subscriber('/mocap_markers_data', MocapData, self.camera_callback)

    def aruco_callback(self, msg):
        self.aruco_pose = Pose()
        self.aruco_pose.position.x = msg.linear.x
        self.aruco_pose.position.y = msg.linear.y
        self.aruco_pose.position.z = msg.linear.z
        quaternion = tf_conversions.transformations.quaternion_from_euler(msg.angular.x, msg.angular.y, msg.angular.z)
        self.aruco_pose.orientation.x = quaternion[0]
        self.aruco_pose.orientation.y = quaternion[1]
        self.aruco_pose.orientation.z = quaternion[2]
        self.aruco_pose.orientation.w = quaternion[3]
        self.calculate_aruco_in_world()

    def camera_callback(self, msg):
        # Assuming camera's ID is 15
        for i in range(len(msg.ids)):
            if msg.ids[i] == 15:
                self.camera_pose = msg.poses[i]
        self.calculate_aruco_in_world()

    def calculate_aruco_in_world(self):
        if self.aruco_pose is None or self.camera_pose is None:
            return

        # Construct the transforms
        camera_in_world = TransformStamped()
        camera_in_world.header.stamp = rospy.Time.now()
        camera_in_world.header.frame_id = "world"
        camera_in_world.child_frame_id = "camera"
        camera_in_world.transform.translation = self.camera_pose.position
        camera_in_world.transform.rotation = self.camera_pose.orientation

        aruco_in_camera = TransformStamped()
        aruco_in_camera.header.stamp = rospy.Time.now()
        aruco_in_camera.header.frame_id = "camera"
        aruco_in_camera.child_frame_id = "aruco"
        aruco_in_camera.transform.translation = self.aruco_pose.position
        aruco_in_camera.transform.rotation = self.aruco_pose.orientation

        # Calculate the aruco marker pose in world frame
        aruco_in_world = self.tf_buffer.transform(aruco_in_camera, "world", rospy.Duration(1.0))
        print(aruco_in_world)

if __name__ == '__main__':
    rospy.init_node('aruco_world_position_node')
    aruco_in_world = ArucoInWorldFrame()
    rospy.spin()
