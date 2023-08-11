import sys
import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist     
 
class Controller(Node):
  def __init__(self):
    super().__init__('Controller')

    self.publisher_ = self.create_publisher(
        Twist, '/cmd_vel', 10
    )

    timer_period = 0.1
    self.timer = self.create_timer(timer_period, self.timer_callback)

  def circle_cmd_vel(self):
    msg = Twist()
    msg.linear.x = 0.2
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.5
    self.publisher_.publish(msg) 

  def timer_callback(self):
     self.circle_cmd_vel()

def main(args=None):
 
    print('Starting circle path node...')
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()