from gpiozero import Motor
from time import sleep

# --- Motor Configuration ---
# Motor(forward, backward, enable)
# For F/B motor: IN3=19, IN4=13, ENB=26
motor_fb = Motor(forward=13, backward=19, enable=26)

# For L/R motor: IN1=20, IN2=21, ENA=16
motor_lr = Motor(forward=20, backward=21, enable=16)

try:
    # --- F/B Motor Tests ---
    print("Motor BACKWARDS - 50% Speed")
    # In gpiozero, motor.backward(speed) takes a value between 0.0 and 1.0
    motor_fb.backward(0.5)
    sleep(3)

    print("Motor FORWARD - 75% Speed")
    motor_fb.forward(0.75)
    sleep(3)

    print("Stopping Motor F/B")
    motor_fb.stop()

    # --- L/R Motor Tests ---
    print("Motor LEFT - 50% Speed")
    motor_lr.backward(0.5) # Based on your original IN1 LOW, IN2 HIGH
    sleep(3)

    print("Stopping Motor L/R")
    motor_lr.stop()
    sleep(1)

    print("Motor RIGHT - 100% Speed")
    motor_lr.forward(1.0) # Based on your original IN1 HIGH, IN2 LOW
    sleep(3)

    print("Stopping Motor L/R")
    motor_lr.stop()
    sleep(2)

except KeyboardInterrupt:
    print("\nScript stopped by user.")
# Note: gpiozero handles cleanup automatically when the script exits!