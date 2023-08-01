import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Pose, PoseArray
from visualization_msgs.msg import Marker

class ArucoListener(Node):
    def __init__(self):
        super().__init__('aruco_listener')
        self.aruco_large_x, self.aruco_large_y, self.aruco_large_z = 0.0, 0.0, 0.0
        self.aruco_small_x, self.aruco_small_y, self.aruco_small_z = 0.0, 0.0, 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.aruco_large_publisher = self.create_publisher(
            Pose, 'aruco_map_large_tf', 10
        )
        self.aruco_small_publisher = self.create_publisher(
            Pose, 'aruco_map_small_tf', 10
        )
        self.aruco_large_marker_publisher = self.create_publisher(
            Marker, 'large_marker', 10
        )
        self.aruco_small_marker_publisher = self.create_publisher(
            Marker, 'small_marker', 10
        )

        self.aruco_poses_large_subscriber = self.create_subscription(
            PoseArray, 'aruco_poses_large', self.aruco_poses_large_callback, 10
        )
        self.aruco_poses_small_subscriber = self.create_subscription(
            PoseArray, 'aruco_poses_small', self.aruco_poses_small_callback, 10
        )

        self.timer_large = self.create_timer(
            0.1, self.timer_large_callback
        )

        self.timer_small = self.create_timer(
            0.1, self.timer_small_callback
        )

    def timer_large_callback(self):
        if self.aruco_large_x and self.aruco_large_y and self.aruco_large_z != 0.0:
            try:
                tf_large = self.tf_buffer.lookup_transform(
                    'map', 'aruco_link_large', rclpy.time.Time()
                )

                large_pose = Pose()
                large_pose.position.x = tf_large.transform.translation.x
                large_pose.position.y = tf_large.transform.translation.y
                large_pose.position.z = tf_large.transform.translation.z
                large_pose.orientation.x = tf_large.transform.rotation.x
                large_pose.orientation.y = tf_large.transform.rotation.y
                large_pose.orientation.z = tf_large.transform.rotation.z
                large_pose.orientation.w = tf_large.transform.rotation.w

                self.aruco_large_publisher.publish(large_pose)

                large_marker = Marker()
                # populate with marker parameters
                #################################
                large_marker.header.frame_id = "map"
                large_marker.ns = 'large_marker'
                large_marker.type = large_marker.SPHERE
                large_marker.action = large_marker.ADD
                large_marker.header.stamp = self.get_clock().now().to_msg()

                # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
                large_marker.type = 2
                large_marker.id = 1

                # Set the scale of the marker
                large_marker.scale.x = 0.1
                large_marker.scale.y = 0.1
                large_marker.scale.z = 0.1

                # Set the color
                large_marker.color.r = 0.0
                large_marker.color.g = 1.0
                large_marker.color.b = 0.0
                large_marker.color.a = 1.0

                # Set the pose of the marker
                large_marker.pose.position.x = large_pose.position.x
                large_marker.pose.position.y = large_pose.position.x
                large_marker.pose.position.z = large_pose.position.x
                large_marker.pose.orientation.x = large_pose.orientation.x
                large_marker.pose.orientation.y = large_pose.orientation.x
                large_marker.pose.orientation.z = large_pose.orientation.x
                large_marker.pose.orientation.w = large_pose.orientation.x

                self.aruco_large_marker_publisher.publish(large_marker)
            except Exception as e:
                self.get_logger().warn("Transform lookup for large aruco failed: {}".format(str(e)))


    def timer_small_callback(self):
        if self.aruco_small_x and self.aruco_small_y and self.aruco_small_z != 0.0:
            try:
                tf_small = self.tf_buffer.lookup_transform(
                    'map', 'aruco_link_small', rclpy.time.Time()
                )

                small_pose = Pose()
                small_pose.position.x = tf_small.transform.translation.x
                small_pose.position.y = tf_small.transform.translation.y
                small_pose.position.z = tf_small.transform.translation.z
                small_pose.orientation.x = tf_small.transform.rotation.x
                small_pose.orientation.y = tf_small.transform.rotation.y
                small_pose.orientation.z = tf_small.transform.rotation.z
                small_pose.orientation.w = tf_small.transform.rotation.w
                self.aruco_small_publisher.publish(small_pose)


                small_marker = Marker()

                # populate with marker parameters
                #################################
                small_marker.header.frame_id = "map"
                small_marker.ns = 'small_marker'
                small_marker.type = small_marker.SPHERE
                small_marker.action = small_marker.ADD
                small_marker.header.stamp = self.get_clock().now().to_msg()

                # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
                small_marker.type = 2
                small_marker.id = 2

                # Set the scale of the marker
                small_marker.scale.x = 0.1
                small_marker.scale.y = 0.1
                small_marker.scale.z = 0.1

                # Set the color
                small_marker.color.r = 0.0
                small_marker.color.g = 1.0
                small_marker.color.b = 0.0
                small_marker.color.a = 1.0

                # Set the pose of the marker
                small_marker.pose.position.x = small_pose.position.x
                small_marker.pose.position.y = small_pose.position.x
                small_marker.pose.position.z = small_pose.position.x
                small_marker.pose.orientation.x = small_pose.orientation.x
                small_marker.pose.orientation.y = small_pose.orientation.x
                small_marker.pose.orientation.z = small_pose.orientation.x
                small_marker.pose.orientation.w = small_pose.orientation.x

                self.aruco_small_marker_publisher.publish(small_marker)
            except Exception as e:
                self.get_logger().warn("Transform lookup for small aruco failed: {}".format(str(e)))


    def aruco_poses_large_callback(self, msg):
        for pose in msg.poses:
            self.aruco_large_x = pose.position.x
            self.aruco_large_y = pose.position.y
            self.aruco_large_z = pose.position.z

    def aruco_poses_small_callback(self, msg):
        for pose in msg.poses:
            self.aruco_small_x = pose.position.x
            self.aruco_small_y = pose.position.y
            self.aruco_small_z = pose.position.z
    
def main(args=None):
    rclpy.init(args=args)
    node = ArucoListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
