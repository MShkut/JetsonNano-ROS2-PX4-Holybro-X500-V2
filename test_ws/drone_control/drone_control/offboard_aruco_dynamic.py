# Import necessary libraries
import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, Timesync, VehicleCommand
from ros2_aruco_interfaces.msg import ArucoParams


class OffboardAruco(Node):
    def __init__(self):
        super().__init__('offboard_aruco')
        # Initialize PID Parameters
        self.kp = 0.3
        self.ki = 0.02 * self.kp
        self.integral = 0.0
        self.control_x, self.control_y = 0.0, 0.0

        self.marker_size = 0.024
        self.aruco_dictionary_id = 'DICT_4X4_1000'

        # Initialize Flags
        self.home_setpoint_set = False
        self.aruco_detected = False
        self.aruco_small_params_set = False

        # Initialize Remaining Variables
        self.offboard_setpoint_counter, self.current_setpoint_index, self.timestamp, self.search_timeout = 0, 0, 0, 0
        self.home_x, self.home_y, self.curr_x, self.curr_y, self.curr_z = 0.0, 0.0, 0.0, 0.0, 0.0
        self.aruco_x, self.aruco_y, self.z, self.yaw = 0.0, 0.0, -1.5, 0.0
        
        #Setpoints must be in NED frame - reverse conversion done during map transform publisher(y and z are made negative)
        #Same must be done in timer_callback for setpoint updater
        self.setpoints = [
            (self.home_x, self.home_y, -1.5, self.yaw), 
            (1.0, 0.0, -1.5, self.yaw), #where you assume the aruco to be, eventually from spot odom data
            (self.aruco_x, -self.aruco_y, -1.5, self.yaw)
        ]
        
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
        self.marker_params_publisher = self.create_publisher(
            ArucoParams, 'marker_params', 10
        )

        #Subscribers
        self.timesync_subscriber = self.create_subscription(
            Timesync, 'fmu/timesync/out', self.timesync_callback, 10
        )
        self.odometry_subscriber = self.create_subscription(
            PoseStamped, '/odometry', self.odometry_callback, 10
        )
        self.aruco_poses_subscriber = self.create_subscription(
            Pose, 'aruco_map_tf',self.aruco_poses_callback, 10
        )


        #Timer
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
        self.get_logger().info('Shutting down ROS node...')
        self.destroy_node()
        rclpy.shutdown()
    
    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('Landing initiated')
        time.sleep(5)
        self.get_logger().info(f'X error: {self.curr_x - self.aruco_x}')
        self.get_logger().info(f'y error: {self.curr_y - self.aruco_y}')
        self.disarm()
    
    def odometry_callback(self, msg):
        if self.home_setpoint_set == False:
            self.home_x = msg.pose.position.x
            self.home_y = -msg.pose.position.y
            self.get_logger().info(str(self.home_x))
            self.get_logger().info(str(self.home_y))
            self.home_setpoint_set = True

        self.curr_x = msg.pose.position.x
        self.curr_y = msg.pose.position.y
        self.curr_z = msg.pose.position.z

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
        msg.x, msg.y, msg.z, msg.yaw = self.setpoints[self.current_setpoint_index]
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
    
    def timesync_callback(self, msg):
        self.timestamp = msg.timestamp

    def aruco_poses_callback(self, msg):
        self.aruco_x = msg.position.x
        self.aruco_y = msg.position.y
        self.aruco_detected = True
    
    def PID_calculate(self, error):
        self.integral += error * 0.1
        output = self.kp * error + self.ki * self.integral
        return output
    
    def update_marker_params(self):
        msg = ArucoParams()
        msg.double_value = self.marker_size
        msg.string_value = self.aruco_dictionary_id
        self.marker_params_publisher.publish(msg)
        self.aruco_small_params_set = True


    def timer_callback(self):
        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

        if self.offboard_setpoint_counter == 10:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)
            self.arm()

        self.publish_offboard_control_mode()
        self.publish_offboard_control()
        self.search_timeout += 1 #100 loops = 10 seconds

        if 1.45 < self.curr_z < 1.55 and self.current_setpoint_index == 0 and self.search_timeout <299:
            self.current_setpoint_index = 1
            self.get_logger().info('Setpoint 1')
            
        if self.aruco_detected == True and self.current_setpoint_index == 1 or self.current_setpoint_index == 2:
            # Use PID controller to adjust setpoint based on error
            self.get_logger().info('Acquiring aruco')
            self.z += 0.009
            self.current_setpoint_index = 2   
            self.control_x = self.PID_calculate(self.curr_x - self.aruco_x)
            self.control_y = self.PID_calculate(self.curr_y - self.aruco_y)
            self.setpoints[2] = (self.aruco_x + self.control_x, -self.aruco_y + self.control_y, self.z, self.yaw)

        if self.aruco_detected == True and self.aruco_small_params_set == False:
                self.update_marker_params()
        
        if self.curr_z < 0.33:
            self.land()
        
        if self.search_timeout >= 300 and self.aruco_detected == False:
            self.current_setpoint_index = 0
            self.get_logger().info('Returning to home')
        
        if self.search_timeout >= 400 and self.aruco_detected == False and self.current_setpoint_index == 0:
            self.land()


def main(args=None):
    print('Starting offboard aruco control node...')
    rclpy.init(args=args)
    offboard_aruco = OffboardAruco()
    rclpy.spin(offboard_aruco)
    offboard_aruco.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
