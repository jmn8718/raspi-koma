from gpiozero import DistanceSensor
from gpiozero.pins.pigpio import PiGPIOFactory
import time

# Initialize factory and sensor
factory = PiGPIOFactory()
sensor1 = DistanceSensor(echo=22, trigger=27, pin_factory=factory)
sensor2 = DistanceSensor(echo=23, trigger=24, pin_factory=factory)

try:
    while True:
        # Distance is in meters, so * 100 for cm
        print(f"Distance 1: {sensor1.distance * 100:.2f} cm")
        print(f"Distance 2: {sensor2.distance * 100:.2f} cm")
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Measurement stopped by user")