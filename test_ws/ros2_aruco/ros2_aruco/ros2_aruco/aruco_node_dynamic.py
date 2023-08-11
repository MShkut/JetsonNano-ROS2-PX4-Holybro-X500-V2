"""
This node locates Aruco AR markers in images and publishes their ids and poses.

Subscriptions:
   /camera/image_raw (sensor_msgs.msg.Image)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)

Published Topics:
    /aruco_poses (geometry_msgs.msg.PoseArray)
       Pose of all detected markers (suitable for rviz visualization)

    /aruco_ID (ros2_aruco_interfaces.msg.ArucoID)
       Provides corresponding marker ids.

Parameters:
    marker_size - size of the markers in meters (default .0625)
    aruco_dictionary_id - dictionary that was used to generate markers (default DICT_5X5_250)
    image_topic - image topic to subscribe to (default /camera/image_raw)
    camera_info_topic - camera info topic to subscribe to (default /camera/camera_info)
    aruco_image_topic - topic name of image published for axes and border identifcation
    aruco_poses_topic - topic name of position data published on self.poses_pub
    aruco_link_name - topic name of aruco link for transforms and mapping (Rviz)
    camera_frame - topic name of camera frame

Author: Nathan Sprague
Edited by: Mitchell Shkut
Version: 9/8/2023
"""

import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import numpy as np
import cv2
import tf_transformations
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseArray, Pose, TransformStamped
from ros2_aruco_interfaces.msg import ArucoID, ArucoParams
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from tf2_ros import TransformBroadcaster
from cv2 import aruco


class ArucoNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("aruco_node")
       # Declare and read parameters
        self.declare_parameter(
            name="marker_size",
            value=0.5,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Size of the markers in meters.",
            ),
        )

        self.declare_parameter(
            name="aruco_dictionary_id",
            value="DICT_ARUCO_ORIGINAL",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Dictionary that was used to generate markers.",
            ),
        )

        self.declare_parameter(
            name="image_topic",
            value="/camera/image_raw",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Image topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="camera_info_topic",
            value="/camera/camera_info",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera info topic to subscribe to.",
            ),
        )
        
        self.declare_parameter(
            name="aruco_image_topic",
            value="aruco_image",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Aruco marker detection image topic",
            ),
        )

        self.declare_parameter(
            name="aruco_poses_topic",
            value="aruco_poses",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Aruco position topic",
            ),
        )

        self.declare_parameter(
            name="aruco_link_name",
            value="aruco_link",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Aruco link name",
            ),
        )

        self.declare_parameter(
            name="camera_frame",
            value="",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera optical frame to use.",
            ),
        )

        self.marker_size = (
            self.get_parameter("marker_size").get_parameter_value().double_value
        )
        self.get_logger().info(f"Marker size: {self.marker_size}")

        self.aruco_dictionary_id = (
            self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value
        )
        self.get_logger().info(f"Marker type: {self.aruco_dictionary_id}")

        self.image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Image topic: {self.image_topic}")

        self.camera_info_topic = (
            self.get_parameter("camera_info_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Image info topic: {self.camera_info_topic}")

        self.camera_frame = (
            self.get_parameter("camera_frame").get_parameter_value().string_value
        )
        self.aruco_image_topic = (
            self.get_parameter("aruco_image_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Aruco image topic: {self.aruco_image_topic}")
        
        self.aruco_poses_topic = (
            self.get_parameter("aruco_poses_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Aruco poses topic: {self.aruco_poses_topic}")

        self.aruco_link_name = (
            self.get_parameter("aruco_link_name").get_parameter_value().string_value
        )
        self.get_logger().info(f"Aruco link name: {self.aruco_link_name}")

        # Make sure we have a valid dictionary id:
        try:
            dictionary_id = cv2.aruco.__getattribute__(self.aruco_dictionary_id)
            if type(dictionary_id) != type(cv2.aruco.DICT_5X5_100):
                raise AttributeError
        except AttributeError:
            self.get_logger().error(
                "bad aruco_dictionary_id: {}".format(self.aruco_dictionary_id)
            )
            options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT")])
            self.get_logger().error("valid options: {}".format(options))

        # Set up subscriptions
        self.info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.info_callback, qos_profile_sensor_data
        )

        self.create_subscription(
            CompressedImage, self.image_topic, self.image_callback, qos_profile_sensor_data
        )

        self.marker_params = self.create_subscription(
            ArucoParams, "marker_params", self.marker_params_callback, qos_profile_sensor_data
        )

        # Set up transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Set up publishers
        self.poses_pub = self.create_publisher(PoseArray, self.aruco_poses_topic, 10)
        self.IDs_pub = self.create_publisher(ArucoID, "aruco_ID", 10)
        #self.img_pub = self.create_publisher(CompressedImage, self.aruco_image_topic, 10)

        # Set up fields for camera parameters
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        self.aruco_dictionary = cv2.aruco.Dictionary_get(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()
        self.bridge = CvBridge()

    def marker_params_callback(self, msg):
        self.marker_size = msg.double_value
        self.aruco_dictionary_id = msg.string_value
        self.get_logger().info(f"Received new marker size: {self.marker_size}")
        self.get_logger().info(f"Received dictionary ID: {self.aruco_dictionary_id}")

        new_marker_param = rclpy.Parameter('marker_size', rclpy.Parameter.Type.DOUBLE, self.marker_size)
        new_dict_param = rclpy.Parameter('aruco_dictionary_id', rclpy.Parameter.Type.STRING, self.aruco_dictionary_id)
        all_new_params = [new_marker_param, new_dict_param]
        self.set_parameters(all_new_params)


    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)
        # Assume that camera parameters will remain the same...
        self.destroy_subscription(self.info_sub)

    def image_callback(self, img_msg):
        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return

        cv_image = self.bridge.compressed_imgmsg_to_cv2(img_msg)
        IDs = ArucoID()
        pose_array = PoseArray()
        
        if self.camera_frame == "":
            IDs.header.frame_id = self.info_msg.header.frame_id
            pose_array.header.frame_id = self.info_msg.header.frame_id
        else:
            IDs.header.frame_id = self.camera_frame
            pose_array.header.frame_id = self.camera_frame

        IDs.header.stamp = img_msg.header.stamp
        pose_array.header.stamp = img_msg.header.stamp

        corners, marker_ids, rejected = cv2.aruco.detectMarkers(
            cv_image, self.aruco_dictionary, parameters=self.aruco_parameters
        )

        self.aruco_image_topic = cv_image

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_link'
        t.child_frame_id = self.aruco_link_name

        if marker_ids is not None:

            if cv2.__version__ > "4.0.0":
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size, self.intrinsic_mat, self.distortion
                )
            else:
                rvecs, tvecs = aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size, self.intrinsic_mat, self.distortion
                )
            for i, marker_id in enumerate(marker_ids):
                pose = Pose()
                pose.position.x = tvecs[i][0][0]
                pose.position.y = tvecs[i][0][1]
                pose.position.z = tvecs[i][0][2]

                t.transform.translation.x = pose.position.x
                t.transform.translation.y = pose.position.y
                t.transform.translation.z = pose.position.z

                rot_matrix = np.eye(4)
                rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                quat = tf_transformations.quaternion_from_matrix(rot_matrix)

                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]


                t.transform.rotation.x = pose.orientation.x
                t.transform.rotation.y = pose.orientation.y
                t.transform.rotation.z = pose.orientation.z
                t.transform.rotation.w = pose.orientation.w
                self.tf_broadcaster.sendTransform(t)

                pose_array.poses.append(pose)
                IDs.marker_ids.append(marker_id[0])
            
            #for i in range(len(marker_ids)):
                #self.aruco_image_topic=cv2.drawFrameAxes(cv_image,self.intrinsic_mat,self.distortion,rvecs[i],tvecs[i],0.01,1)

            self.poses_pub.publish(pose_array)
            self.IDs_pub.publish(IDs)

    # These are all required for publishing an image with the border and axes of the marker
    # You may need to switch back to an uncompressed image for this to work, I have not tested
        #frame_markers = cv2.aruco.drawDetectedMarkers(self.aruco_image_topic, corners)
        #self.aruco_image_topic = self.bridge.cv2_to_imgmsg(frame_markers)
        #self.img_pub.publish(self.bridge.cv2_to_imgmsg(self.aruco_image_topic, "bgr8"))

def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
