import RPi.GPIO as GPIO
import time
import os
import subprocess

from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306

# --- Configuration ---
BUTTON_PIN = 7  # Physical Pin 7 (GPIO 4)
OLED_WIDTH = 128
OLED_HEIGHT = 32

# --- Setup OLED ---
try:
    serial = i2c(port=1, address=0x3C)
    device = ssd1306(serial, width=OLED_WIDTH, height=OLED_HEIGHT)
except Exception as e:
    print(f"OLED not found: {e}")
    device = None

# --- Setup GPIO ---
GPIO.setmode(GPIO.BOARD)
# YwRobot is Active High; pull_up_down=GPIO.PUD_OFF uses the module's resistors
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_OFF)

def kill_start_script():
  try:
    subprocess.run(["pkill", "-f", "scripts/pi_start.py"])
  except:
    pass

def show_message(line1, line2 = ""):
  with canvas(device) as draw:
    draw.rectangle(device.bounding_box, outline="white")
    draw.text((4,2), line1, fill="white")
    draw.text((4,18), line2, fill="white")

def shutdown_sequence():
  kill_start_script()
  print("shutdown started...")
  show_message("System Halt", "Shutting down")
  time.sleep(3)
  device.clear()
  os.system("sudo shutdown -h now")

try:
  print("Monitoring button on Pin 7. Press to shutdown.")
  while True:
    # Check if button is pressed (High)
    if GPIO.input(BUTTON_PIN) == GPIO.HIGH:
      # Simple debounce: wait 1 second to make sure it's a real press
      time.sleep(1)
      if GPIO.input(BUTTON_PIN) == GPIO.HIGH:
        shutdown_sequence()
        break
    time.sleep(0.5)

except KeyboardInterrupt:
  GPIO.cleanup()
