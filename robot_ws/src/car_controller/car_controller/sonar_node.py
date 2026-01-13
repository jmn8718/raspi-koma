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
        self.declare_parameter('right_trigger_pin', 27)
        self.declare_parameter('right_echo_pin', 22)
        self.declare_parameter('left_trigger_pin', 24)
        self.declare_parameter('left_echo_pin', 23)

        r_trig = self.get_parameter('right_trigger_pin').value
        r_echo = self.get_parameter('right_echo_pin').value
        l_trig = self.get_parameter('left_trigger_pin').value
        l_echo = self.get_parameter('left_echo_pin').value

        try:
            self.sensor_right = DistanceSensor(
                echo=r_echo, trigger=r_trig, pin_factory=self.factory, max_distance=4.0
            )
            self.sensor_left = DistanceSensor(
                echo=l_echo, trigger=l_trig, pin_factory=self.factory, max_distance=4.0
            )
            self.get_logger().info(f"Sonar sensors initialized (L:{l_echo}/{l_trig}, R:{r_echo}/{r_trig})")
        except Exception as e:
            self.get_logger().error(f"Hardware Init Error: {e}")

        # 3. Setup ROS 2 Publishers (Topics updated for clarity)
        self.pub_left = self.create_publisher(Range, 'ultrasonic/front_left', 10)
        self.pub_right = self.create_publisher(Range, 'ultrasonic/front_right', 10)

        # 4. Create Timer (10Hz = 0.1s interval)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Sonar Node updated with URDF frame names.")

    def create_range_msg(self, frame_id, distance_m, timestamp):
        """Helper to build a standard ROS 2 Range message."""
        msg = Range()
        msg.header.stamp = timestamp
        msg.header.frame_id = frame_id
        msg.radiation_type = Range.ULTRASOUND
        # ~25 degrees (matches your physical rotation)
        msg.field_of_view = 0.436 
        msg.min_range = 0.02     # 2 cm
        msg.max_range = 4.0      # 4 meters
        
        # If distance is max (no echo), gpiozero returns max_distance
        if distance_m >= msg.max_range:
            msg.range = float('inf')  # Indicate no echo detected
        else:
            msg.range = float(distance_m)
        return msg

    def timer_callback(self):
        try:
            # Read distances in meters
            dist_l = self.sensor_left.distance
            # Small 20ms gap so echoes can clear
            time.sleep(0.02)
            dist_r = self.sensor_right.distance
            # self.get_logger().debug(f"Left: {dist_l*100:.1f}cm | Right: {dist_r*100:.1f}cm")

            now = self.get_clock().now().to_msg()
            l_msg = self.create_range_msg('front_left_sonar', dist_l, now)
            r_msg = self.create_range_msg('front_right_sonar', dist_r, now)
            
            self.pub_left.publish(l_msg)
            self.pub_right.publish(r_msg)

        except Exception as e:
            self.get_logger().warn(f"Failed to read sonar: {e}")

    def stop_sensors(self):
        """Crucial: Shut down the sensors to stop background threads"""
        self.get_logger().info("Stopping Sonar background threads...")
        if hasattr(self, 'sensor_left'): self.sensor_left.close()
        if hasattr(self, 'sensor_right'): self.sensor_right.close()

def main(args=None):
    rclpy.init(args=args)
    node = DualSonarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Sonar node stopping...")
    finally:
        # 1. Manually close the sensors to stop gpiozero/pigpio threads
        # This prevents the 'NoneType' object has no attribute 'send' error
        node.stop_sensors()
        
        # 2. Destroy the node
        node.destroy_node()
        
        # 3. Shutdown ROS 2 safely (prevents 'rcl_shutdown already called')
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()