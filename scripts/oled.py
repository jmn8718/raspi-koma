import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import board
import busio
from adafruit_ssd1306 import SSD1306_I2C
from PIL import Image, ImageDraw, ImageFont
import subprocess
import time
import sys

class HybridOled:
    def __init__(self):
        # I2C Hardware Setup for 128x32
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.oled = SSD1306_I2C(128, 32, i2c)
            self.font = ImageFont.load_default()
        except Exception as e:
            print(f"Hardware Error: {e}")
            sys.exit(1)
        
        # Data storage
        self.linear_pwr = 0
        self.steer_pwr = 0
        self.is_moving = False
        self.ros_active = False

    def get_sys_info(self):
        """Fetch IP and CPU usage for standby mode"""
        try:
            ip = subprocess.check_output("hostname -I | cut -d' ' -f1", shell=True).decode().strip()
            if not ip: ip = "No Network"
            cpu = subprocess.check_output("top -bn1 | grep 'Cpu(s)' | awk '{print $2 + $4}'", shell=True).decode().strip()
            return ip, cpu
        except:
            return "Error", "0.0"

    def draw_display(self):
        """Render the screen based on current mode"""
        self.oled.fill(0)
        image = Image.new("1", (self.oled.width, self.oled.height))
        draw = ImageDraw.Draw(image)

        if self.ros_active:
            # --- ROBOT MODE (Compact Side-by-Side) ---
            status = "MOVE" if self.is_moving else "IDLE"
            draw.text((0, 0), f"ROS: {status}", font=self.font, fill=255)
            
            # Drive Line (Label + Bar)
            draw.text((0, 10), f"DRV:{self.linear_pwr}%", font=self.font, fill=255)
            draw.rectangle((60, 12, 127, 18), outline=255, fill=0)
            bar_w_d = int((self.linear_pwr / 100.0) * 67)
            draw.rectangle((60, 12, 60 + bar_w_d, 18), outline=255, fill=255)

            # Steer Line (Label + Bar)
            draw.text((0, 20), f"STR:{self.steer_pwr}%", font=self.font, fill=255)
            draw.rectangle((60, 22, 127, 28), outline=255, fill=0)
            bar_w_s = int((self.steer_pwr / 100.0) * 67)
            draw.rectangle((60, 22, 60 + bar_w_s, 28), outline=255, fill=255)
        else:
            # --- STANDBY MODE (Tight 0/10/20 spacing) ---
            ip, cpu = self.get_sys_info()
            draw.text((0, 0), "MODE: STANDBY", font=self.font, fill=255)
            draw.text((0, 10), f"IP: {ip}", font=self.font, fill=255)
            draw.text((0, 20), f"CPU Load: {cpu}%", font=self.font, fill=255)

        self.oled.image(image)
        self.oled.show()

    def clear_screen(self):
        """Blackout the screen to prevent burn-in"""
        self.oled.fill(0)
        self.oled.show()

class OledRosSubscriber(Node):
    def __init__(self, parent):
        super().__init__('oled_ros_node')
        self.parent = parent
        # Match this to your actual topic name from 'ros2 topic list'
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.vel_cb, 10)

    def vel_cb(self, msg):
        self.parent.linear_pwr = int(abs(msg.linear.x) * 100)
        self.parent.steer_pwr = int(abs(msg.angular.z) * 100)
        self.parent.is_moving = (self.parent.linear_pwr > 0 or self.parent.steer_pwr > 0)

def main():
    display = HybridOled()
    
    # Ensure ROS is initialized once
    if not rclpy.ok():
        rclpy.init()

    try:
        while rclpy.ok():
            node = OledRosSubscriber(display)
            
            # Check if there is any motor activity/publishers
            # This triggers the mode switch
            if node.count_publishers('/cmd_vel') > 0:
                display.ros_active = True
            else:
                display.ros_active = False
                # If IDLE, reset the bars
                display.linear_pwr = 0
                display.steer_pwr = 0
                display.is_moving = False

            # Process ROS callbacks for 0.5 seconds
            rclpy.spin_once(node, timeout_sec=0.5)
            display.draw_display()
            
            node.destroy_node()

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Script Error: {e}")
    finally:
        display.clear_screen()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()