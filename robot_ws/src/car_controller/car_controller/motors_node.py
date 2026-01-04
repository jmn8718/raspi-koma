import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import PWMOutputDevice, DigitalOutputDevice
from gpiozero.pins.pigpio import PiGPIOFactory
import time

class DualDCMotorController(Node):
    def __init__(self):
        super().__init__('motor_node')
        
        # --- Parameters ---
        self.declare_parameter('drive_pwm_pin', 16)
        self.declare_parameter('drive_in3_pin', 21)
        self.declare_parameter('drive_in4_pin', 20)
        self.declare_parameter('steer_pwm_pin', 26)
        self.declare_parameter('steer_in1_pin', 19)
        self.declare_parameter('steer_in2_pin', 13)

        drive_pwm = self.get_parameter('drive_pwm_pin').value
        drive_in3 = self.get_parameter('drive_in3_pin').value
        drive_in4 = self.get_parameter('drive_in4_pin').value
        steer_pwm = self.get_parameter('steer_pwm_pin').value
        steer_in1 = self.get_parameter('steer_in1_pin').value
        steer_in2 = self.get_parameter('steer_in2_pin').value

        factory = PiGPIOFactory()
        
        # --- Drive Motor ---
        self.drive_pwm = PWMOutputDevice(drive_pwm, pin_factory=factory)
        self.drive_in3 = DigitalOutputDevice(drive_in3, pin_factory=factory)
        self.drive_in4 = DigitalOutputDevice(drive_in4, pin_factory=factory)
        
        # --- Steering Motor ---
        self.steer_pwm = PWMOutputDevice(steer_pwm, pin_factory=factory)
        self.steer_in1 = DigitalOutputDevice(steer_in1, pin_factory=factory)
        self.steer_in2 = DigitalOutputDevice(steer_in2, pin_factory=factory)
        
        # Ramping variables
        self.target_steer = 0.0
        self.current_steer = 0.0
        self.ramp_step = 0.1  # How much to change per timer cycle
        
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.listener_callback, 10)
        
        # Timer to handle the smooth ramping (50Hz / 0.02s)
        self.create_timer(0.02, self.ramp_timer_callback)
        
        self.get_logger().info("Motor Node: 2-Step Steering & Ramping enabled")

    def listener_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        # self.get_logger().debug(f"Linear x {linear_x} | z {angular_z}")

        # 1. Drive Logic
        if linear_x > 0:
            self.drive_in3.on(); self.drive_in4.off()
            self.drive_pwm.value = min(abs(linear_x), 1.0)
        elif linear_x < 0:
            self.drive_in3.off(); self.drive_in4.on()
            self.drive_pwm.value = min(abs(linear_x), 1.0)
        else:
            self.drive_pwm.value = 0

        # 2. Steering Logic (2-Step target setting)
        abs_z = abs(angular_z)

        # For now disable as it does not apply much turn
        # if abs_z > 0.5:
        #     self.target_steer = 1.0  # FULL
        # elif abs_z > 0.1:
        #     self.target_steer = 0.5  # HALF
        #     if abs_z > 0.5:
        #     self.target_steer = 1.0  # FULL

        if abs_z > 0.1:
            self.target_steer = 1.0  # FULL
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
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()