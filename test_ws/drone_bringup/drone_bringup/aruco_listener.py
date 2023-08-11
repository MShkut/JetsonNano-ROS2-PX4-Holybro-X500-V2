import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Pose, PoseArray
#from visualization_msgs.msg import Marker

class ArucoListener(Node):
    def __init__(self):
        super().__init__('aruco_listener')
        self.aruco_x, self.aruco_y, self.aruco_z = 0.0, 0.0, 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        self.aruco_small_publisher = self.create_publisher(
            Pose, 'aruco_map_tf', 10
        )
        
        # For visualizing aruco marker transform position in Rviz2
        '''self.aruco_small_marker_publisher = self.create_publisher(
            Marker, 'aruco_marker', 10
        )'''

        self.aruco_poses_small_subscriber = self.create_subscription(
            PoseArray, 'aruco_poses', self.aruco_poses_callback, 10
        )
        self.timer_small = self.create_timer(
            0.1, self.timer_small_callback
        )

    def timer_small_callback(self):
        if self.aruco_x and self.aruco_y and self.aruco_z != 0.0:
            try:
                tf_small = self.tf_buffer.lookup_transform(
                    'map', 'aruco_link', rclpy.time.Time()
                )

                pose = Pose()
                pose.position.x = tf_small.transform.translation.x
                pose.position.y = tf_small.transform.translation.y
                pose.position.z = tf_small.transform.translation.z
                pose.orientation.x = tf_small.transform.rotation.x
                pose.orientation.y = tf_small.transform.rotation.y
                pose.orientation.z = tf_small.transform.rotation.z
                pose.orientation.w = tf_small.transform.rotation.w
                self.aruco_small_publisher.publish(pose)

                # For visualizing aruco marker transform position in Rviz2
                '''
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
                small_marker.pose.position.y = small_pose.position.y
                small_marker.pose.position.z = small_pose.position.z
                small_marker.pose.orientation.x = small_pose.orientation.x
                small_marker.pose.orientation.y = small_pose.orientation.y
                small_marker.pose.orientation.z = small_pose.orientation.z
                small_marker.pose.orientation.w = small_pose.orientation.w
                

                self.aruco_small_marker_publisher.publish(aruco_marker)
                '''
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
