import multiprocessing
import RPi.GPIO as GPIO
from sensor_ultrasound import initialize_sensor

GPIO.setmode(GPIO.BCM)

# GPIO pin for Trigger of first sensor
TRIG_PIN_1 = 27 
# GPIO pin for Echo of first sensor
ECHO_PIN_1 = 22 

# GPIO pin for Trigger of second sensor
TRIG_PIN_2 = 23 
# GPIO pin for Echo of second sensor
ECHO_PIN_2 = 24 

def sensor_1():
  try:
    initialize_sensor('1', TRIG_PIN_1, ECHO_PIN_1)
  except KeyboardInterrupt:
    print("Sensor 1 stopped.")
    pass

def sensor_2():
  try:
    initialize_sensor('2', TRIG_PIN_2, ECHO_PIN_2)
  except KeyboardInterrupt:
    print("Sensor 2 stopped.")
    pass

try:
  print("Measuring distance. Press Ctrl+C to stop.")
  # Create process objects targeting the functions
  p1 = multiprocessing.Process(target=sensor_1)
  p2 = multiprocessing.Process(target=sensor_2)

  # Start the processes
  p1.start()
  p2.start()

  # Wait for both processes to complete
  p1.join()
  p2.join()

  print("Measurement stopped.")
  GPIO.cleanup()

except KeyboardInterrupt:
  print("Stopping all sensors...")
  pass