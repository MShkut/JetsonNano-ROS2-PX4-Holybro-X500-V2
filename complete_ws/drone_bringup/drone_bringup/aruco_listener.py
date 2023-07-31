import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Pose, PoseArray

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

        self.aruco_poses_large_subscriber = self.create_subscription(
            PoseArray, 'aruco_poses_large',self.aruco_poses_large_callback, 10
        )
        self.aruco_poses_small_subscriber = self.create_subscription(
            PoseArray, 'aruco_poses_large', self.aruco_poses_small_callback, 10
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
