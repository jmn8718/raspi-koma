import time
import psutil
import socket
import subprocess
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306

# --- Configuration ---
OLED_WIDTH = 128
OLED_HEIGHT = 32
MAX_CHARS = 20 

# --- Initialization ---
serial = i2c(port=1, address=0x3c)
device = ssd1306(serial, width=OLED_WIDTH, height=OLED_HEIGHT)
device.contrast(10)

def is_hotspot_active():
    """Checks if hostapd is currently running the Access Point."""
    try:
        # Check if the hostapd service is active
        status = subprocess.check_output(["systemctl", "is-active", "hostapd"]).decode().strip()
        return status == "active"
    except:
        return False

def get_client_count():
    """Counts connected devices by querying the WiFi driver station dump."""
    try:
        # 'iw' lists all stations. We count the occurrences of the word 'Station'
        cmd = "iw dev wlan0 station dump | grep -c Station"
        count = subprocess.check_output(cmd, shell=True).decode().strip()
        return f"[{count}]" if count.isdigit() else "[0]"
    except:
        return "[0]"

def get_interface_ip(ifname):
    interfaces = psutil.net_if_addrs()
    if ifname in interfaces:
        for addr in interfaces[ifname]:
            if addr.family == socket.AF_INET:
                ip = addr.address
                if len(ip) > MAX_CHARS:
                    return f"*{ip[-(MAX_CHARS-1):]}" 
                return ip
    return "None"

heartbeat = True

try:
    while True:
        eth_ip = get_interface_ip('eth0')
        wlan_ip = get_interface_ip('wlan0')
        
        # Determine Hotspot status and client count
        if is_hotspot_active():
            wifi_label = "AP"
            clients = get_client_count()
        else:
            wifi_label = "W"
            clients = ""

        with canvas(device) as draw:
            # Line 1: Ethernet
            draw.text((0, 0),  f"E: {eth_ip}", fill="white")
            # Line 2: WiFi (Label + IP + [Client Count])
            # We use a f-string to combine the mode, IP, and the number of connected devices
            draw.text((0, 16), f"{wifi_label}: {wlan_ip} {clients}", fill="white")

            # Heartbeat Pixel
            if heartbeat:
                draw.rectangle((125, 0, 127, 2), fill="white")
            
        heartbeat = not heartbeat
        time.sleep(2)

except Exception:
    device.clear()
except KeyboardInterrupt:
    device.clear()