import time
import os
import psutil
import socket

from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306

# --- Configuration ---
OLED_WIDTH = 128
OLED_HEIGHT = 32
OLED_WIDTH_HALF = OLED_WIDTH / 2
OLED_HEIGHT_HALF = OLED_HEIGHT / 2
HEIGHT_MARGIN = 2
WIDTH_MARGIN = 2

# --- 1. Initialization ---
serial = i2c(port=1, address=0x3c)
device = ssd1306(serial, width=OLED_WIDTH, height=OLED_HEIGHT)
device.contrast(10)

# Toggle variable for the heartbeat
heartbeat_state = True

def get_ip():
  s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  try:
    s.connect(("10.255.255.255", 1))
    ip = s.getsockname()[0]
  except Exception:
    ip = "127.0.0.1"
  finally:
    s.close()
  if ip == "":
    ip = "NO IP"
  return ip

def get_temp():
  temp = os.popen("vcgencmd measure_temp").readline()
  return temp.replace("temp=", "").replace("'C\n", " C")

def show_message(line1, line2 = ""):
  with canvas(device) as draw:
    draw.rectangle(device.bounding_box, outline="white")
    draw.text((WIDTH_MARGIN,HEIGHT_MARGIN), line1, fill="white")
    draw.text((WIDTH_MARGIN,OLED_HEIGHT_HALF + HEIGHT_MARGIN), line2, fill="white")

show_message("BOOTING", "RASPI-KOMA ...")
time.sleep(5)

def draw_ip(include_icon = False):
  start_position = 0
  if include_icon:
    # --- Draw Network Icon (Small signal bars) ---
    draw.rectangle((0, 8, 2, 10), fill="white")
    draw.rectangle((4, 5, 6, 10), fill="white")
    draw.rectangle((8, 2, 10, 10), fill="white")
    start_position = 15
  draw.text((start_position, 0), get_ip(), fill="white")

def draw_info(include_icon = False):
  DRAW_Y_POS = 18
  start_position = 0
  if include_icon:
    # --- Draw Thermometer Icon ---
    # Circle base
    draw.ellipse((2, DRAW_Y_POS + 7, 7, 30), outline="white", fill="white")
    # Stem
    draw.rectangle((3, DRAW_Y_POS, 6, 26), outline="white", fill="white")
    start_position = 15
  
  # --- Draw CPU Temp ---
  draw.text((start_position, DRAW_Y_POS), get_temp(), fill="white")

  mem = psutil.virtual_memory().percent

  if include_icon:
    DRAW_BAR_Y = DRAW_Y_POS + 3
    DRAW_BAR_X_POS_START = OLED_WIDTH_HALF
    PERCENTAGE_MODIFIER = 4
    DRAW_BAR_X_POS_END = int(100/PERCENTAGE_MODIFIER) + DRAW_BAR_X_POS_START
    DRAW_BAR_X_POS_MEMORY = int(mem/PERCENTAGE_MODIFIER) + DRAW_BAR_X_POS_START
    print(DRAW_BAR_X_POS_END)
    print(DRAW_BAR_X_POS_MEMORY)
    DRAW_MEM_TEXT = DRAW_BAR_X_POS_END + 4
    draw.text((DRAW_MEM_TEXT, DRAW_Y_POS), f"{mem}%", fill="white")
    draw.rectangle((DRAW_BAR_X_POS_START, DRAW_BAR_Y, DRAW_BAR_X_POS_END, 28), outline="white") # Outline
    draw.rectangle((DRAW_BAR_X_POS_START, DRAW_BAR_Y, DRAW_BAR_X_POS_MEMORY, 28), fill="white") # Fill based on %
  else:
    draw.text((OLED_WIDTH_HALF, DRAW_Y_POS), f"{mem}%", fill="white")

try:
  while True:
    with canvas(device) as draw:
      # Draw Dashboard
      draw_ip()
      draw_info()

      # --- Heartbeat Pixel (Top Right Corner) ---
      if heartbeat_state:
        # Draw a small 2x2 square pulse
        draw.rectangle((125, 0, 127, 2), fill="white")
            
      # Toggle the state for the next blink
      heartbeat_state = not heartbeat_state
    time.sleep(2)
except KeyboardInterrupt:
  device.clear()
  pass
