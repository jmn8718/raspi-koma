from flask import Flask, render_template, request, Response, jsonify
import RPi.GPIO as GPIO
import os

# --- GPIO Setup ---
ENB, IN3, IN4 = 26, 19, 13
ENA, IN1, IN2 = 16, 20, 21

GPIO.setmode(GPIO.BCM)
GPIO.setup([ENB, IN3, IN4, ENA, IN1, IN2], GPIO.OUT)
p_drive = GPIO.PWM(ENB, 100); p_steer = GPIO.PWM(ENA, 100)
p_drive.start(0); p_steer.start(0)

app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/stats')
def stats():
    # Gets CPU temp
    res = os.popen('vcgencmd measure_temp').readline()
    temp = res.replace("temp=","").replace("'C\n","")
    return jsonify(temp=temp)

@app.route('/move')
def move():
    speed = float(request.args.get('speed', 0))
    turn = float(request.args.get('turn', 0))
    
    # Drive
    if speed > 0:
        GPIO.output(IN3, 1); GPIO.output(IN4, 0)
        p_drive.ChangeDutyCycle(speed)
    elif speed < 0:
        GPIO.output(IN3, 0); GPIO.output(IN4, 1)
        p_drive.ChangeDutyCycle(abs(speed))
    else:
        p_drive.ChangeDutyCycle(0)
    
    # Steer
    if turn > 0:
        GPIO.output(IN1, 0); GPIO.output(IN2, 1)
        p_steer.ChangeDutyCycle(100)
    elif turn < 0:
        GPIO.output(IN1, 1); GPIO.output(IN2, 0)
        p_steer.ChangeDutyCycle(100)
    else:
        p_steer.ChangeDutyCycle(0)
    return "ok"

@app.route('/brake')
def brake():
    GPIO.output([IN1, IN2, IN3, IN4], 1)
    p_drive.ChangeDutyCycle(100); p_steer.ChangeDutyCycle(100)
    return "ok"

if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=5005, threaded=True)
    finally:
        GPIO.cleanup()