import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from mpu6050 import mpu6050
import math

class MPU6050Publisher(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.sensor = mpu6050(0x68)
        self.timer = self.create_timer(0.1, self.timer_callback) # 10Hz
        self.get_logger().info("MPU-6050 Node Started")

    def timer_callback(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Read data
        accel = self.sensor.get_accel_data()
        gyro = self.sensor.get_gyro_data()

        # Accelerometer (m/s^2) - MPU-6050 returns Gs by default, convert to m/s^2
        msg.linear_acceleration.x = accel['x'] * 9.80665
        msg.linear_acceleration.y = accel['y'] * 9.80665
        msg.linear_acceleration.z = accel['z'] * 9.80665

        # Gyroscope (rad/sec) - Convert degrees/sec to radians/sec
        msg.angular_velocity.x = math.radians(gyro['x'])
        msg.angular_velocity.y = math.radians(gyro['y'])
        msg.angular_velocity.z = math.radians(gyro['z'])

        # Note: MPU-6050 does not provide orientation (quaternion) natively 
        # without complex DMP processing. For now, we leave orientation identity.
        msg.orientation.w = 1.0

        self.publisher_.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(MPU6050Publisher())
    rclpy.shutdown()