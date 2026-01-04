import time
import psutil
import socket
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306
from luma.core.virtual import viewport

# --- Configuration ---
OLED_WIDTH = 128
OLED_HEIGHT = 32
VIRTUAL_WIDTH = 256  # Extra width to accommodate long IPs

# --- Initialization ---
serial = i2c(port=1, address=0x3c)
device = ssd1306(serial, width=OLED_WIDTH, height=OLED_HEIGHT)
# Viewport is our "window" that can move over a larger 256x32 virtual canvas
virtual = viewport(device, width=VIRTUAL_WIDTH, height=OLED_HEIGHT)
device.contrast(10)

def get_interface_ip(ifname):
    interfaces = psutil.net_if_addrs()
    if ifname in interfaces:
        for addr in interfaces[ifname]:
            if addr.family == socket.AF_INET:
                return f"{ifname.upper()}: {addr.address}"
    return f"{ifname.upper()}: None"

# Heartbeat state
heartbeat = True
x_offset = 0
direction = 1

try:
    while True:
        eth_text = get_interface_ip('eth0')
        wlan_text = get_interface_ip('wlan0')

        # Draw onto the virtual canvas
        with canvas(virtual) as draw:
            draw.text((0, 0), eth_text, fill="white")
            draw.text((0, 16), wlan_text, fill="white")
            
            # Heartbeat pixel (stays fixed relative to the text)
            if heartbeat:
                draw.point((OLED_WIDTH - 2, 0), fill="white")

        # Scroll logic: Move the viewport left/right
        # We only scroll if the text is likely to be cut off
        virtual.set_position((x_offset, 0))
        
        # Adjust scroll position (moving 2 pixels at a time)
        if x_offset >= 60: direction = -1  # Max scroll reached
        if x_offset <= 0: direction = 1    # Back to start
        
        x_offset += (direction * 2)
        heartbeat = not heartbeat
        
        time.sleep(0.1) # Faster refresh for smooth scrolling

except KeyboardInterrupt:
    device.clear()
    pass