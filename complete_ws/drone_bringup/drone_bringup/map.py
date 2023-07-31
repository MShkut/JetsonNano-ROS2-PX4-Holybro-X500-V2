import rclpy
import math

from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, quaternion_multiply


class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, 'odometry', 10) #create odometry publisher for easy topic visualizations
        self.subscriber = self.create_subscription(VehicleOdometry, 'fmu/vehicle_odometry/out', self.listener_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

    def listener_callback(self, msg):
        q_orig = (msg.q[0], msg.q[1], msg.q[2], msg.q[3])
        rotate = quaternion_from_euler(math.pi, 0, 0) #for the drone working previously it was (math.pi, 0, 0)
        q_new = quaternion_multiply(rotate, q_orig)
        self.publish_map(msg.x, msg.y, msg.z, float(q_new[0]), float(q_new[1]), float(q_new[2]), float(q_new[3]))
 
    def publish_map(self, x, y, z, q_x, q_y, q_z, q_w):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odometry"
        msg.pose.position.x = x
        msg.pose.position.y = -y
        msg.pose.position.z = -z
        msg.pose.orientation.x = q_x
        msg.pose.orientation.y = -q_y
        msg.pose.orientation.z = -q_z
        msg.pose.orientation.w = q_w
        self.publisher_.publish(msg)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        t.transform.translation.x = x
        t.transform.translation.y = -y
        t.transform.translation.z = -z
        t.transform.rotation.x = q_z
        t.transform.rotation.y = q_y
        t.transform.rotation.z = -q_x
        t.transform.rotation.w = -q_w
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
