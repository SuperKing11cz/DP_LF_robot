import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        
        # Subscribe to 5 IR sensors (left to right: lls_ir, lcs_ir, cs_ir, rcs_ir, rrs_ir)
        self.create_subscription(LaserScan, 'lls_ir', self.lls_callback, 10)
        self.create_subscription(LaserScan, 'lcs_ir', self.lcs_callback, 10)
        self.create_subscription(LaserScan, 'cs_ir', self.cs_callback, 10)
        self.create_subscription(LaserScan, 'rcs_ir', self.rcs_callback, 10)
        self.create_subscription(LaserScan, 'rrs_ir', self.rrs_callback, 10)
        
        # Publisher for robot velocity
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Initialize sensor readings
        self.sensor_values = [float('inf')] * 5  # [lls, lcs, cs, rcs, rrs]

        # PID Controller variables
        self.Kp = 0.8  # Proportional gain
        self.Ki = 0.0  # Integral gain (set to 0 if not needed)
        self.Kd = 0.1  # Derivative gain
        self.previous_error = 0.0
        self.integral = 0.0
        
        # Control loop timer (10 Hz)
        self.create_timer(0.1, self.control_loop)
    
    def lls_callback(self, msg):
        self.sensor_values[0] = min(msg.ranges)

    def lcs_callback(self, msg):
        self.sensor_values[1] = min(msg.ranges)

    def cs_callback(self, msg):
        self.sensor_values[2] = min(msg.ranges)

    def rcs_callback(self, msg):
        self.sensor_values[3] = min(msg.ranges)

    def rrs_callback(self, msg):
        self.sensor_values[4] = min(msg.ranges)

    def control_loop(self):
        cmd = Twist()
        threshold = 0.15  # Distance threshold for detecting the line
        
        # Binary activation: 1 if the sensor detects the line (below threshold), else 0
        #sensor_activations = [1 if val < threshold else 0 for val in self.sensor_values]
        for index in range(0, len(sensor_activations)):
            if sensor_activations[index] < threshold:
                sensor_activations[index] = 1
            else:
                sensor_activations[index] = 0

        # Weights for position error calculation
        weights = [-2, -1, 0, 1, 2]

        # Clear cases: extreme turns if only one sensor is active
        if sensor_activations == [1, 0, 0, 0, 0]:
            error = -2.0  # Strong LEFT turn
        elif sensor_activations == [0, 1, 0, 0, 0]:
            error = -1.0  # Mild LEFT turn
        elif sensor_activations == [0, 0, 1, 0, 0]:
            error = 0.0  # Centered
        elif sensor_activations == [0, 0, 0, 1, 0]:
            error = 1.0  # Mild RIGHT turn
        elif sensor_activations == [0, 0, 0, 0, 1]:
            error = 2.0  # Strong RIGHT turn
        else:
            # Weighted average error calculation
            total_activation = sum(sensor_activations)
            if total_activation > 0:
                weighted_sum = sum(weight * val for weight, val in zip(weights, sensor_activations))
                error = weighted_sum / total_activation
            else:
                error = self.previous_error  # No detection, maintain previous error
        # PID Calculations
        self.integral += error
        derivative = error - self.previous_error
        self.previous_error = error
        
        correction = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        
        # Publish velocity command
        cmd.linear.x = 1.0
        cmd.angular.z = correction
        
        self.vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
