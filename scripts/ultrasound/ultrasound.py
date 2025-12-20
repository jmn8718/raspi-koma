import RPi.GPIO as GPIO
from sensor_ultrasound import initialize_sensor

GPIO.setmode(GPIO.BCM)

# GPIO pin for Trigger of first sensor
TRIG_PIN_1 = 23 
# GPIO pin for Echo of first sensor
ECHO_PIN_1 = 24 

try:
    print("Measuring distance. Press Ctrl+C to stop.")
    initialize_sensor('1', TRIG_PIN_1, ECHO_PIN_1)

except KeyboardInterrupt:
  GPIO.cleanup()
  pass