import rclpy
from rclpy.node import Node
from threading import Timer
from std_msgs.msg import Header
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, Timesync, VehicleCommand,VehicleOdometry
from tf_transformations import euler_from_quaternion

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')
        self.offboard_setpoint_counter = 0
        self.timestamp = 0
        self.odometry_received = False
        self.time = 0
        self.zero_x = 0.0
        self.zero_y = 0.0
        self.zero_z = 0.0
        self.zero_yaw = 0.0
        self.setpoints = [
            (self.zero_x, self.zero_y, -1.75, self.zero_yaw),
            (self.zero_x, self.zero_y, self.zero_z, self.zero_yaw)
        ]
        self.current_setpoint_index = 0


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

        self.z = Timer(1, self.destroy)
        self.t = Timer(15, self.update_setpoint)
        self.l = Timer(20, self.land)
        self.d = Timer(30, self.disarm)
        self.z.start()
        self.t.start()
        self.l.start()
        self.d.start()
        self.timer = self.create_timer(0.1, self.timer_callback)
        
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info('Disarm command sent')
        self.d.cancel()

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
        self.get_logger().info(str(self.zero_x))
        self.get_logger().info(str(self.zero_y))
        self.get_logger().info(str(self.zero_z))
        self.get_logger().info(str(self.zero_yaw))
        
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

    def update_setpoint(self):
        self.current_setpoint_index = 1
        self.t.cancel()

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.l.cancel()    

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
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
