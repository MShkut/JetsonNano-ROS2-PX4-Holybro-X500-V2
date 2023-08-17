import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Pose, PoseArray
from visualization_msgs.msg import Marker

class ArucoListener(Node):
    def __init__(self):
        super().__init__('aruco_listener')
        self.aruco_x, self.aruco_y, self.aruco_z = 0.0, 0.0, 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        self.aruco_publisher = self.create_publisher(
            Pose, 'aruco_map_tf', 10
        )
        
        # For visualizing aruco marker transform position in Rviz2
        self.aruco_small_marker_publisher = self.create_publisher(
            Marker, 'aruco_marker', 10
        )

        self.aruco_poses_subscriber = self.create_subscription(
            PoseArray, 'aruco_poses', self.aruco_poses_callback, 10
        )
        self.timer = self.create_timer(
            0.1, self.timer_callback
        )

    def timer_callback(self):
        if self.aruco_x and self.aruco_y and self.aruco_z != 0.0:
            try:
                tf = self.tf_buffer.lookup_transform(
                    'map', 'aruco_link', rclpy.time.Time()
                )

                pose = Pose()
                pose.position.x = tf.transform.translation.x
                pose.position.y = tf.transform.translation.y
                pose.position.z = tf.transform.translation.z
                pose.orientation.x = tf.transform.rotation.x
                pose.orientation.y = tf.transform.rotation.y
                pose.orientation.z = tf.transform.rotation.z
                pose.orientation.w = tf.transform.rotation.w
                self.aruco_publisher.publish(pose)

                # For visualizing aruco marker transform position in Rviz2
                
                marker = Marker()

                # populate with marker parameters
                #################################
                marker.header.frame_id = "map"
                marker.ns = 'small_marker'
                marker.type = marker.SPHERE
                marker.action = marker.ADD
                marker.header.stamp = self.get_clock().now().to_msg()

                # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
                marker.type = 2
                marker.id = 2

                # Set the scale of the marker
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1

                # Set the color
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0

                # Set the pose of the marker
                marker.pose.position.x = pose.position.x
                marker.pose.position.y = pose.position.y
                marker.pose.position.z = pose.position.z
                marker.pose.orientation.x = pose.orientation.x
                marker.pose.orientation.y = pose.orientation.y
                marker.pose.orientation.z = pose.orientation.z
                marker.pose.orientation.w = pose.orientation.w
                

                self.aruco_small_marker_publisher.publish(marker)
                
            except Exception as e:
                error=True

    def aruco_poses_callback(self, msg):
        for pose in msg.poses:
            self.aruco_x = pose.position.x
            self.aruco_y = pose.position.y
            self.aruco_z = pose.position.z
    
def main(args=None):
    rclpy.init(args=args)
    node = ArucoListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
