''' Does not work without a global position estimate (Needs GPS)
    All flight modes which are used for autonomous flights need the same
    The only option left is offboard control with a custom landing sequence
    This program may oneday become relevant but it's been 4 years as a known issue'''


import rclpy
from rclpy.node import Node
from px4_msgs.msg import LandingTargetPose, VehicleCommand, Timesync, VehicleOdometry
from geometry_msgs.msg import PoseArray, Pose

class LandingPositionPublisher(Node):
    def __init__(self):
        super().__init__('landing_position_publisher')

        self.timestamp = 0
        self.counter = 0
        self.home_x, self.home_y, self.home_z = 0.0, 0.0, 0.0
        self.aruco_large_x, self.aruco_large_y, self.aruco_large_z = 0.0, 0.0, 0.0
        self.aruco_small_x, self.aruco_small_y, self.aruco_small_z = 0.0, 0.0, 0.0

        self.landing_target_pose_publisher = self.create_publisher(
            LandingTargetPose, 'fmu/landing_target_pose/in', 10
        )
        self.com_publisher = self.create_publisher(
            VehicleCommand, 'fmu/vehicle_command/in', 10
        )
        self.timesync_subscriber = self.create_subscription(
            Timesync, 'fmu/timesync/out', self.timesync_callback, 10
        )

        self.aruco_poses_large_subscriber = self.create_subscription(
            PoseArray, 'aruco_poses_large',self.aruco_poses_large_callback, 10
        )
        self.aruco_poses_small_subscriber = self.create_subscription(
            PoseArray, 'aruco_poses_large', self.aruco_poses_small_callback, 10
        )

        self.timer = self.create_timer(
            1.0, self.timer_callback
        )

    def landing_target(self):
        msg = LandingTargetPose()
        msg.is_static = True
        msg.abs_pos_valid = True
        
        if self.aruco_large_x is not None and self.counter < 4:
            msg.x_abs = self.aruco_large_x 
            msg.y_abs = self.aruco_large_y
            msg.z_abs = 0.5
            self.counter +=1
            self.get_logger().info('LArge Aruco Detected')
        if self.aruco_small_x is not None and self.counter >= 4:
            msg.x_abs = self.aruco_small_x 
            msg.y_abs = self.aruco_small_y
            msg.z_abs = self.aruco_small_z
            self.get_logger().info('Small Aruco Detected')

        self.landing_target_pose_publisher.publish(msg)

    def vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.timestamp
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.com_publisher.publish(msg)

    def timesync_callback(self, msg):
        self.timestamp = msg.timestamp

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


    def timer_callback(self): 
        self.vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_PRECLAND)
        self.landing_target()


def main(args=None):
    rclpy.init(args=args)
    landing_position_publisher = LandingPositionPublisher()
    rclpy.spin(landing_position_publisher)
    landing_position_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
