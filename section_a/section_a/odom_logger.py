import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

def yaw_from_quaternion(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)

class OdomLogger(Node):
    def __init__(self):
        super().__init__('odom_logger')
        self.sub = self.create_subscription(Odometry, '/odom', self.callback, 10)

    def callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.get_logger().info(f"x={x:.3f}, y={y:.3f}, yaw={yaw:.3f} rad")

def main(args=None):
    rclpy.init(args=args)
    node = OdomLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()