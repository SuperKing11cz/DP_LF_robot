import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class StopCarNode(Node):
    def __init__(self):
        super().__init__('stop_car')
        
        # Define a distance threshold for stopping (meters)
        self.stop_distance = 0.5 
        
        # Subscribe to the LiDAR topic (change if needed)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        # Publisher rychlosti
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info("StopCarNode started. Monitoring LiDAR...")

    def lidar_callback(self, msg):
        """Processes LiDAR data and stops the car if an obstacle is too close."""
        if len(msg.ranges) == 0:
            return 
        
        # 0 je index paprsku lidaru (je jen jeden takze 0)
        distance = msg.ranges[0]

        self.get_logger().info(f"Vzdalenost od prekazky: {distance:.2f}m")

        # Stop if too close
        if distance < self.stop_distance:
            self.stop_car()

    def stop_car(self):
        """Publishes a zero velocity command to stop the car."""
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(stop_msg)
        self.get_logger().warn("Obstacle detected! Stopping the car.")

def main(args=None):
    rclpy.init(args=args)
    node = StopCarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
