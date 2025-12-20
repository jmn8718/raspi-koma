import RPi.GPIO as GPIO
import time

def measure_distance(trigger_pin, echo_pin):
    # Ensure trigger is low
    GPIO.output(trigger_pin, False)
    time.sleep(0.5)

    # Send 10us pulse to trigger
    GPIO.output(trigger_pin, True)
    time.sleep(0.00001)
    GPIO.output(trigger_pin, False)

    # Wait for echo start
    while GPIO.input(echo_pin) == 0:
        pulse_start = time.time()

    # Wait for echo end
    while GPIO.input(echo_pin) == 1:
        pulse_end = time.time()

    # Calculate pulse duration
    pulse_duration = pulse_end - pulse_start

    # Calculate distance (speed of sound is 34300 cm/s)
    # Distance in cm
    distance = pulse_duration * 17150 

    return round(distance, 2)

def initialize_sensor(id, trigger_pin, echo_pin):
    print(f"Setting up ultrasonic sensor {id} ...")
    GPIO.setup(trigger_pin, GPIO.OUT)
    GPIO.setup(echo_pin, GPIO.IN)
    print(f"Trigger Pin: {trigger_pin}, Echo Pin: {echo_pin}")

    print(f"Ultrasonic sensor {id} ready.")
    while True:
        dist = measure_distance(trigger_pin, echo_pin)
        print(f"Sensor {id} | Distance: {dist} cm")
        time.sleep(0.2)