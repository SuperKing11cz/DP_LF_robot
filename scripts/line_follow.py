import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        
        # Subscribers for IR sensors
        self.create_subscription(LaserScan, 'ir1', self.ir1_callback, 10)
        self.create_subscription(LaserScan, 'ir2', self.ir2_callback, 10)
        self.create_subscription(LaserScan, 'ir3', self.ir3_callback, 10)
        self.create_subscription(LaserScan, 'ir4', self.ir4_callback, 10)
        
        # Publisher for robot velocity
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Sensor readings
        self.ir1 = float('inf')
        self.ir2 = float('inf')
        self.ir3 = float('inf')
        self.ir4 = float('inf')
        
        # Timer to control robot movement
        self.create_timer(0.1, self.control_loop)
    
    def ir1_callback(self, msg):
        self.ir1 = min(msg.ranges)
    
    def ir2_callback(self, msg):
        self.ir2 = min(msg.ranges)
    
    def ir3_callback(self, msg):
        self.ir3 = min(msg.ranges)
    
    def ir4_callback(self, msg):
        self.ir4 = min(msg.ranges)
    
    def control_loop(self):
        cmd = Twist()
        
        # Thresholds for detecting the line
        line_detected = 0.15
        no_line = 0.16
        
        ir1_detects = self.ir1 < line_detected
        ir2_detects = self.ir2 < line_detected
        ir3_detects = self.ir3 < line_detected
        ir4_detects = self.ir4 < line_detected
        
        if ir2_detects and ir3_detects:
            # Move forward
            cmd.linear.x = 0.2
            cmd.angular.z = 0.0
        elif ir1_detects:
            # Turn left
            cmd.linear.x = 0.1
            cmd.angular.z = -0.3
        elif ir4_detects:
            # Turn right
            cmd.linear.x = 0.1
            cmd.angular.z = 0.3
        else:
            # Stop if no valid readings
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0        
        self.vel_pub.publish(cmd)

        if self.ir1<line_detected:
            self.get_logger().info(f"IR1: {"1"}")
        else:
            self.get_logger().info(f"IR1: {"0"}")
        if self.ir2<line_detected:
            self.get_logger().info(f"IR2: {"1"}")
        else:
            self.get_logger().info(f"IR2: {"0"}")
        if self.ir3<line_detected:
            self.get_logger().info(f"IR3: {"1"}")
        else:
            self.get_logger().info(f"IR3: {"0"}")
        if self.ir4<line_detected:
            self.get_logger().info(f"IR4: {"1"}")
        else:
            self.get_logger().info(f"IR4: {"0"}")

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

