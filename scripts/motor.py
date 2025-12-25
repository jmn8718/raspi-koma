import RPi.GPIO as GPIO
from time import sleep

# F/B motor
ENB = 26
IN3 = 19
IN4 = 13

# L/R motor
IN2 = 21
IN1 = 20
ENA = 16

GPIO.setmode(GPIO.BCM)
GPIO.setup(ENB, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)


GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)

# Initialize PWM on ENB at 100Hz
pwm_b = GPIO.PWM(ENB, 100)
pwm_b.start(0) # Start with motor off

pwm_a = GPIO.PWM(ENA, 100)
pwm_a.start(0) # Start with motor off

try:
  print("Motor BACKWARDS (Clockwise) - 50% Speed")
  GPIO.output(IN3, GPIO.HIGH)
  GPIO.output(IN4, GPIO.LOW)
  pwm_b.ChangeDutyCycle(50)
  sleep(3)

  print("Motor FORWARD (Counter-Clockwise) - 75% Speed")
  GPIO.output(IN3, GPIO.LOW)
  GPIO.output(IN4, GPIO.HIGH)
  pwm_b.ChangeDutyCycle(75)
  sleep(3)

  print("Stopping Motor F/B")
  pwm_b.ChangeDutyCycle(0)
  GPIO.output(IN3, GPIO.LOW)
  GPIO.output(IN4, GPIO.LOW)

  print("Motor LEFT - 50%")
  GPIO.output(IN1, GPIO.LOW)
  GPIO.output(IN2, GPIO.HIGH)
  pwm_a.ChangeDutyCycle(50)
  sleep(3)

  print("Stopping Motor L/R")
  pwm_a.ChangeDutyCycle(0)
  GPIO.output(IN1, GPIO.LOW)
  GPIO.output(IN2, GPIO.LOW)
  sleep(1)

  print("Motor RIGHT - 50%")
  GPIO.output(IN1, GPIO.HIGH)
  GPIO.output(IN2, GPIO.LOW)
  pwm_a.ChangeDutyCycle(100)
  sleep(3)

  print("Stopping Motor L/R")
  pwm_a.ChangeDutyCycle(0)
  GPIO.output(IN1, GPIO.LOW)
  GPIO.output(IN2, GPIO.LOW)
  sleep(2)

except KeyboardInterrupt:
  pass
finally:
  pwm_b.stop()
  GPIO.cleanup()
  print("GPIO Cleaned up.")