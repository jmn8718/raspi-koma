import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from gpiozero import DistanceSensor
from gpiozero.pins.pigpio import PiGPIOFactory
import time

class DualSonarNode(Node):
    def __init__(self):
        super().__init__('sonar_node')
        
        # 1. Initialize Hardware Factory (pigpio)
        try:
            self.factory = PiGPIOFactory()
            self.get_logger().info("Successfully connected to pigpiod daemon.")
        except Exception as e:
            self.get_logger().error(f"FATAL: Could not connect to pigpiod: {e}")
            return

        # 2. Configure Sensors
        # Right Sensor: Trig 27, Echo 22
        # Left Sensor: Trig 24, Echo 23
        try:
            self.sensor_right = DistanceSensor(
                echo=22, trigger=27, pin_factory=self.factory, max_distance=4.0
            )
            self.sensor_left = DistanceSensor(
                echo=23, trigger=24, pin_factory=self.factory, max_distance=4.0
            )
            self.get_logger().info("Sonar sensors initialized (L:22/27, R:23/24)")
        except Exception as e:
            self.get_logger().error(f"Hardware Init Error: {e}")

        # 3. Setup ROS 2 Publishers
        self.pub_left = self.create_publisher(Range, 'ultrasonic/left', 10)
        self.pub_right = self.create_publisher(Range, 'ultrasonic/right', 10)

        # 4. Create Timer (10Hz = 0.1s interval)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def create_range_msg(self, frame_id, distance_m):
        """Helper to build a standard ROS 2 Range message."""
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.52  # ~30 degrees in radians
        msg.min_range = 0.02     # 2 cm
        msg.max_range = 4.0      # 4 meters
        
        # If distance is max (no echo), gpiozero returns max_distance
        msg.range = float(distance_m)
        return msg

    def timer_callback(self):
        try:
            # Read distances in meters
            dist_l = self.sensor_left.distance
            dist_r = self.sensor_right.distance

            # Publish messages
            self.pub_left.publish(self.create_range_msg('sonar_left_link', dist_l))
            self.pub_right.publish(self.create_range_msg('sonar_right_link', dist_r))

            # Optional: Log data to console for debugging
            # self.get_logger().info(f"Left: {dist_l*100:.1f}cm | Right: {dist_r*100:.1f}cm")

        except Exception as e:
            self.get_logger().warn(f"Failed to read sonar: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DualSonarNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Sonar node stopping...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()