import lgpio
import time
import os
import subprocess

from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306

# --- Configuration ---
BUTTON_PIN = 4  # Physical Pin 7 is GPIO 4 (BCM)
OLED_WIDTH = 128
OLED_HEIGHT = 32

# --- Setup OLED ---
try:
    serial = i2c(port=1, address=0x3C)
    device = ssd1306(serial, width=OLED_WIDTH, height=OLED_HEIGHT)
except Exception as e:
    print(f"OLED not found: {e}")
    device = None

# --- Setup lgpio ---
h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_input(h, BUTTON_PIN)

def kill_start_script():
  try:
    subprocess.run(["pkill", "-f", "scripts/pi_start.py"])
  except:
    pass

def show_message(line1, line2=""):
    if device:
        with canvas(device) as draw:
            draw.rectangle(device.bounding_box, outline="white")
            draw.text((4, 2), line1, fill="white")
            draw.text((4, 18), line2, fill="white")

def shutdown_sequence():
    kill_start_script()
    print("shutdown started...")
    show_message("System Halt", "Shutting down")
    time.sleep(3)
    if device:
        device.clear()
    
    # We don't close the handle here anymore; the 'finally' block handles it
    os.system("sudo shutdown -h now")

try:
    print(f"Monitoring button on BCM Pin {BUTTON_PIN}. Press to shutdown.")
    while True:
        if lgpio.gpio_read(h, BUTTON_PIN) == 1:
            time.sleep(1)
            if lgpio.gpio_read(h, BUTTON_PIN) == 1:
                shutdown_sequence()
                break # Exiting the loop triggers the 'finally' block
        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nManual exit...")

finally:
    # This block runs NO MATTER WHAT (button press or Ctrl+C)
    # Check if handle 'h' exists before trying to close it
    if 'h' in locals():
        try:
            lgpio.gpiochip_close(h)
            print("GPIO handle closed.")
        except:
            pass
