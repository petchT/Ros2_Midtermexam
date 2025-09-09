import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CirclePublisher(Node):
    def __init__(self):
        super().__init__('circle_publisher')

        # Declare parameters
        self.declare_parameter('linear_speed', 0.20)  # m/s
        self.declare_parameter('radius', 0.50)        # m

        v = float(self.get_parameter('linear_speed').value)
        r = float(self.get_parameter('radius').value)
        omega = v / r if r != 0 else 0.0

        self.v = v
        self.omega = omega

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.10, self.timer_callback)

        self.get_logger().info(
            f"CirclePublisher started: v={self.v:.3f} m/s, r={r:.3f} m → ω={self.omega:.3f} rad/s"
        )

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.v
        msg.angular.z = self.omega
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CirclePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()