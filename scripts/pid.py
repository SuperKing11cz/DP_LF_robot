import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        
        # Subscribers for 4 IR sensors (assumed physical order left-to-right: ir1, ir2, ir3, ir4)
        self.create_subscription(LaserScan, 'ir1', self.ir1_callback, 10)
        self.create_subscription(LaserScan, 'ir2', self.ir2_callback, 10)
        self.create_subscription(LaserScan, 'ir3', self.ir3_callback, 10)
        self.create_subscription(LaserScan, 'ir4', self.ir4_callback, 10)
        
        # Publisher for robot velocity
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Initialize sensor readings
        self.ir1 = float('inf')
        self.ir2 = float('inf')
        self.ir3 = float('inf')
        self.ir4 = float('inf')
        
        # PID Controller variables
        self.Kp = 0.2  # Proportional gain
        self.Ki = 0.0  # Integral gain (unused, set to 0 if not needed)
        self.Kd = 0.1  # Derivative gain
        self.previous_error = 0.0
        self.integral = 0.0
        
        # Control loop timer (10 Hz)
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
        threshold = 0.15  # Distance threshold for detecting the line
        
        # Binary activation for each sensor: 1 if the sensor detects the line (i.e. reading below threshold), else 0.
        s1 = 1 if self.ir1 < threshold else 0
        s2 = 1 if self.ir2 < threshold else 0
        s3 = 1 if self.ir3 < threshold else 0
        s4 = 1 if self.ir4 < threshold else 0
        
        # First, apply explicit sensor logic for clear cases.
        # For example, if only the leftmost sensor is active, force a sharp right turn.
        if s1 == 1 and s2 == 0 and s3 == 0 and s4 == 0:
            error = -2.0
        elif s2 == 1 and s1 == 0 and s3 == 0 and s4 == 0:
            error = -1.0
        elif s3 == 1 and s1 == 0 and s2 == 0 and s4 == 0:
            error = 1.0
        elif s4 == 1 and s1 == 0 and s2 == 0 and s3 == 0:
            error = 2.0
        else:
            # Otherwise, use weighted-average error from all active sensors.
            # Sensor layout (left-to-right): ir1, ir2, ir3, ir4.
            sensor_values = [s1, s2, s3, s4]
            weights = [-2, -1, 1, 2]
            total_activation = sum(sensor_values)
            if total_activation > 0:
                weighted_sum = sum(weight * val for weight, val in zip(weights, sensor_values))
                error = weighted_sum / total_activation
            else:
                error = self.previous_error  # No detection, maintain previous error
        
        # PID Calculations
        self.integral += error
        derivative = error - self.previous_error
        self.previous_error = error
        
        correction = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        
        # Command settings: constant forward speed with angular velocity adjusted by PID correction.
        cmd.linear.x = 0.2
        cmd.angular.z = correction  # Negative sign to turn in the correct direction
        
        self.vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()