import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import PWMOutputDevice, DigitalOutputDevice
from gpiozero.pins.pigpio import PiGPIOFactory
import time

class DualDCMotorController(Node):
    def __init__(self):
        super().__init__('motor_node')
        
        factory = PiGPIOFactory()
        
        # --- Drive Motor ---
        self.drive_pwm = PWMOutputDevice(26, pin_factory=factory)
        self.drive_in3 = DigitalOutputDevice(19, pin_factory=factory)
        self.drive_in4 = DigitalOutputDevice(13, pin_factory=factory)
        
        # --- Steering Motor ---
        self.steer_pwm = PWMOutputDevice(16, pin_factory=factory)
        self.steer_in1 = DigitalOutputDevice(20, pin_factory=factory)
        self.steer_in2 = DigitalOutputDevice(21, pin_factory=factory)
        
        # Ramping variables
        self.target_steer = 0.0
        self.current_steer = 0.0
        self.ramp_step = 0.1  # How much to change per timer cycle
        
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.listener_callback, 10)
        
        # Timer to handle the smooth ramping (50Hz / 0.02s)
        self.create_timer(0.02, self.ramp_timer_callback)
        
        self.get_logger().info("Motor Node: 2-Step Steering & Ramping enabled")

    def listener_callback(self, msg):
        # 1. Drive Logic
        linear_x = msg.linear.x
        if linear_x > 0:
            self.drive_in3.on(); self.drive_in4.off()
            self.drive_pwm.value = min(abs(linear_x), 1.0)
        elif linear_x < 0:
            self.drive_in3.off(); self.drive_in4.on()
            self.drive_pwm.value = min(abs(linear_x), 1.0)
        else:
            self.drive_pwm.value = 0

        # 2. Steering Logic (2-Step target setting)
        angular_z = msg.angular.z
        abs_z = abs(angular_z)

        if abs_z > 0.5:
            self.target_steer = 1.0  # FULL
        elif abs_z > 0.1:
            self.target_steer = 0.5  # HALF
        else:
            self.target_steer = 0.0  # CENTER

        # Direction setting
        if angular_z > 0.1:
            self.steer_in1.on(); self.steer_in2.off()
        elif angular_z < -0.1:
            self.steer_in1.off(); self.steer_in2.on()
        else:
            self.steer_in1.off(); self.steer_in2.off()

    def ramp_timer_callback(self):
        # Smoothly move current_steer toward target_steer
        if self.current_steer < self.target_steer:
            self.current_steer = min(self.current_steer + self.ramp_step, self.target_steer)
        elif self.current_steer > self.target_steer:
            self.current_steer = max(self.current_steer - self.ramp_step, self.target_steer)
        
        self.steer_pwm.value = self.current_steer

def main(args=None):
    rclpy.init(args=args)
    node = DualDCMotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.drive_pwm.value = 0
        node.steer_pwm.value = 0
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()