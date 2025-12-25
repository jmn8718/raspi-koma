import RPi.GPIO as GPIO
import sys
import tty
import termios
import select
import time

# --- Pin Definitions ---
ENB, IN3, IN4 = 26, 19, 13  # Drive
ENA, IN1, IN2 = 16, 20, 21  # Steering

# --- GPIO Setup ---
GPIO.setmode(GPIO.BCM)
GPIO.setup([ENB, IN3, IN4, ENA, IN1, IN2], GPIO.OUT)

pwm_drive = GPIO.PWM(ENB, 100)
pwm_steer = GPIO.PWM(ENA, 100)
pwm_drive.start(0)
pwm_steer.start(0)

def get_key():
    """Reads a single keypress from the terminal without waiting for Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        # Wait up to 0.1 seconds for a key
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
            # Handle arrow keys (which are multi-character escape sequences)
            if key == '\x1b':
                key += sys.stdin.read(2)
            return key
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def stop_all():
    pwm_drive.ChangeDutyCycle(0)
    pwm_steer.ChangeDutyCycle(0)
    GPIO.output([IN3, IN4, IN1, IN2], GPIO.LOW)

# New variable to track timing
last_key_time = time.time()
stop_delay = 0.6  # 200ms grace period to bridge the auto-repeat gap

print("--- RC CAR TERMINAL CONTROL (STUTTER-FIXED) ---")
print("Use Arrows to Drive | SPACE to Brake | 'Q' to Quit")

try:
    while True:
        key = get_key()

        if key:
            last_key_time = time.time() # Reset the timer because we have input
            
            if key == 'q' or key == 'Q':
                break
            
            elif key == '\x1b[A': # UP
                GPIO.output(IN3, GPIO.HIGH); GPIO.output(IN4, GPIO.LOW)
                pwm_drive.ChangeDutyCycle(80)
            elif key == '\x1b[B': # DOWN
                GPIO.output(IN3, GPIO.LOW); GPIO.output(IN4, GPIO.HIGH)
                pwm_drive.ChangeDutyCycle(80)
            elif key == '\x1b[D': # LEFT
                GPIO.output(IN1, GPIO.HIGH); GPIO.output(IN2, GPIO.LOW)
                pwm_steer.ChangeDutyCycle(100)
            elif key == '\x1b[C': # RIGHT
                GPIO.output(IN1, GPIO.LOW); GPIO.output(IN2, GPIO.HIGH)
                pwm_steer.ChangeDutyCycle(100)
            elif key == ' ': # SPACE
                GPIO.output([IN3, IN4, IN1, IN2], GPIO.HIGH)
                pwm_drive.ChangeDutyCycle(100)
                pwm_steer.ChangeDutyCycle(100)

        else:
            # Only stop if we haven't seen a key for more than 'stop_delay'
            if time.time() - last_key_time > stop_delay:
                stop_all()

except Exception as e:
    print(f"Error: {e}")

finally:
    stop_all()
    pwm_drive.stop()
    pwm_steer.stop()
    GPIO.cleanup()
    print("\nGPIO Cleaned and Script Ended.")