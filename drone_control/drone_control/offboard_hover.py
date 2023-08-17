import rclpy
from rclpy.node import Node
from threading import Timer
from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, Timesync, VehicleCommand,VehicleOdometry
from tf_transformations import euler_from_quaternion

class OffboardHover(Node):
    def __init__(self):
        super().__init__('offboard_hover')
        self.offboard_setpoint_counter = 0
        self.timestamp = 0
        self.odometry_received = False
        self.time = 0
        self.zero_x, self.zero_y, self.zero_z, self.zero_yaw = 0.0, 0.0, 0.0, 0.0
 

        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, 'fmu/offboard_control_mode/in', 10
        )
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, 'fmu/trajectory_setpoint/in', 10
        )
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, 'fmu/vehicle_command/in', 10
        )


        self.timesync_subscriber = self.create_subscription(
            Timesync, 'fmu/timesync/out', self.timesync_callback, 10
        )
        self.initial_zero_setpoint_subscriber = self.create_subscription(
            VehicleOdometry, 'fmu/vehicle_odometry/out', self.zero_setpoint_setter, 10
        )


        self.z = Timer(0.1, self.destroy)
        self.z.start()
        self.timer = self.create_timer(0.1, self.timer_callback)
        
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info('Arm command sent')

    def destroy(self):
        self.destroy_subscription(self.initial_zero_setpoint_subscriber)
        self.z.cancel()

    def zero_setpoint_setter(self, msg):
        orientation_q = [msg.q[0], msg.q[1], msg.q[2], msg.q[3]]
        orientation = euler_from_quaternion(orientation_q)
        self.zero_yaw = orientation[2]
        self.zero_x = msg.x
        self.zero_y = msg.y
        self.zero_z = msg.z
        
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
        msg.x = self.zero_x  
        msg.y = self.zero_y
        msg.z = -1.5
        msg.yaw = self.zero_yaw
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

    def timer_callback(self):
        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

        if self.offboard_setpoint_counter == 10:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)
            self.arm()

        self.publish_offboard_control_mode()
        self.publish_offboard_control()


    def timesync_callback(self, msg):
        self.timestamp = msg.timestamp

def main(args=None):
    print('Starting offboard hover node...')
    rclpy.init(args=args)
    offboard_hover = OffboardHover()
    rclpy.spin(offboard_hover)
    offboard_hover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
