import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, Timesync, VehicleCommand,VehicleOdometry
from tf_transformations import euler_from_quaternion

class OffboardAruco(Node):
    def __init__(self):
        super().__init__('offboard_aruco')

        self.offboard_setpoint_counter = 0
        self.setpoint_counter_1, self.setpoint_counter_2, self.setpoint_counter_3, self.setpoint_counter_4 = 0.0, 0.0, 0.0, 0.0
        self.timestamp = 0
        self.home_x, self.home_y, self.home_z, self.home_yaw = 0.0, 0.0, 0.0, 0.0
        self.curr_x, self.curr_y, self.curr_z, self.curr_yaw = 0.0, 0.0, 0.0, 0.0
        self.aruco_large_x, self.aruco_large_y, self.aruco_large_z = 0.0, 0.0, 0.0
        self.aruco_small_x, self.aruco_small_y, self.aruco_small_z = 0.0, 0.0, 0.0
        self.setpoints = [
            (self.home_x, self.home_y, -1.5,), 
            (self.home_x, self.home_y, self.home_z), 
            (self.aruco_large_x, self.aruco_large_y, -1.5), 
            (self.aruco_large_x, self.aruco_large_y, -1.0),
            (self.aruco_small_x, self.aruco_small_y, -0.5),
        ]
        self.current_setpoint_index = 0

        #Publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, 'fmu/offboard_control_mode/in', 10
        )
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, 'fmu/trajectory_setpoint/in', 10
        )
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, 'fmu/vehicle_command/in', 10
        )

        #Subscribers
        self.timesync_subscriber = self.create_subscription(
            Timesync, 'fmu/timesync/out', self.timesync_callback, 10
        )
        self.home_setpoint_subscriber = self.create_subscription(
            VehicleOdometry, 'fmu/vehicle_odometry/out', self.home_setpoint_setter, 10
        )
        self.odometry_subscriber = self.create_subscription(
            VehicleOdometry, 'fmu/vehicle_odometry/out', self.odometry_callback, 10
        )
        self.aruco_poses_large_subscriber = self.create_subscription(
            PoseArray, 'aruco_poses_large',self.aruco_poses_large_callback, 10
        )
        self.aruco_poses_small_subscriber = self.create_subscription(
            PoseArray, 'aruco_poses_large', self.aruco_poses_small_callback, 10
        )

        #Timers
        self.timer = self.create_timer(
            0.1, self.timer_callback
        )
        

    #Functions    
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info('Disarm command sent')

    def home_setpoint_setter(self, msg):
        orientation_q = [msg.q[0], msg.q[1], msg.q[2], msg.q[3]]
        orientation = euler_from_quaternion(orientation_q)
        self.home_yaw = orientation[2]
        self.home_x = msg.x
        self.home_y = msg.y
        self.home_z = msg.z
        self.get_logger().info(str(self.home_x))
        self.get_logger().info(str(self.home_y))
        self.get_logger().info(str(self.home_z))
        self.get_logger().info(str(self.home_yaw))
        self.destroy_subscription(self.home_setpoint_subscriber)
    
    def odometry_callback(self, msg):
        orientation_q = [msg.q[0], msg.q[1], msg.q[2], msg.q[3]]
        orientation = euler_from_quaternion(orientation_q)
        self.curr_yaw = orientation[2]
        self.curr_x = msg.x
        self.curr_y = msg.y
        self.curr_z = msg.z

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.timestamp
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_control_mode_publisher.publish(msg)

    def publish_offboard_control(self):
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp
        msg.x, msg.y, msg.z = self.setpoints[self.current_setpoint_index]
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
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
        self.vehicle_command_publisher.publish(msg)

    def update_setpoint(self, index):
        self.current_setpoint_index = index

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

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('landing initiated')

    def timer_callback(self):
        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

        if self.offboard_setpoint_counter == 10:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)
            self.arm()

        self.publish_offboard_control_mode()
        self.publish_offboard_control()

        if self.aruco_large_x != 0.0 and self.current_setpoint_index == 0 and self.curr_z < -1.4:
            self.update_setpoint(2)
            self.get_logger().info('Large aruco setpoint 1 acquired')

        if abs(self.aruco_large_x) < 0.2 and abs(self.aruco_large_y) < 0.2 and -1.55 < self.curr_z < -1.45 and self.current_setpoint_index == 2:
            self.update_setpoint(3)
            self.get_logger().info('Large aruco setpoint 2 acquired')

        if abs(self.aruco_large_x) < 0.1 and abs(self.aruco_large_y) < 0.1 and -1.05 < self.curr_z < -0.95 and self.current_setpoint_index == 3:
            self.update_setpoint(4)
            self.get_logger().info('Small aruco setpoint 1 acquired')

        if abs(self.aruco_large_x) < 0.05 and abs(self.aruco_large_y) < 0.05 and -0.55 < self.curr_z < -0.45 and self.current_setpoint_index == 4:
            self.land()


    def timesync_callback(self, msg):
        self.timestamp = msg.timestamp

def main(args=None):
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_aruco = OffboardAruco()
    rclpy.spin(offboard_aruco)
    offboard_aruco.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
