from flask import Flask, render_template, request, jsonify
import RPi.GPIO as GPIO
from adafruit_servokit import ServoKit
import time
import threading
import signal
import os
import subprocess

process = None

relay_pin = 14  

GPIO.setmode(GPIO.BCM)    
GPIO.setup(relay_pin, GPIO.OUT)

kit = ServoKit(channels=16)
kit.servo[0].set_pulse_width_range(500, 2500) 
current_angle = 90
min_angle, max_angle = 0, 180
kit.servo[0].angle = current_angle

def fire():
	GPIO.output(relay_pin, GPIO.HIGH)  
	time.sleep(0.5)                    
	GPIO.output(relay_pin, GPIO.LOW)

def run_sniper():
    global process
    if process is None or process.poll() is not None:
        process = subprocess.Popen(
            ['python3', 'test_sniper.py'],
            preexec_fn=os.setsid  # run in new process group
        )

try:
	GPIO.output(relay_pin, GPIO.LOW)
			

	app = Flask(__name__)

	@app.route("/")
	def home():
		return render_template("index.html")

	@app.route("/move")
	def move():
		global current_angle
		direction = request.args.get("dir")
		print(f"Move command received: {direction}")
		if direction == 'right':
			current_angle += 10
			if min_angle <= current_angle <= max_angle:
				kit.servo[0].angle = current_angle
				kit.servo[0].angle = current_angle
				return jsonify({"movement": -10})
			else: current_angle -= 10
		elif direction == 'left':
			current_angle -= 10
			if min_angle <= current_angle <= max_angle:
				kit.servo[0].angle = current_angle
				kit.servo[0].angle = current_angle
				return jsonify({"movement": 10})
			else: current_angle += 10
		return jsonify({"movement": 0})

	@app.route("/fire")
	def fire_gun():
		print("FIRE command received")
		GPIO.output(relay_pin, GPIO.HIGH)
		return "FIRING"

	@app.route("/stopFire")
	def stop_gun():
		print("STOP command recieved")
		GPIO.output(relay_pin, GPIO.LOW)
		return "STOPPED"
	
	@app.route("/manual")
	def manual():
		return render_template("manual.html")
	
	@app.route("/auto")
	def auto():
		return render_template("auto.html")
	
	@app.route("/index")
	def index():
		return render_template("index.html")
	
	@app.route("/start_auto")
	def start_auto():
		thread = threading.Thread(target=run_sniper)
		thread.start()
		return "Running Sniper"
	
	@app.route("/stop_auto")
	def stop_sniper():
		global process
		if process and process.poll() is None:
			os.killpg(os.getpgid(process.pid), signal.SIGTERM)  # kill entire group
			process = None

	if __name__ == "__main__":
		app.run(host="0.0.0.0", port=5000) 
finally:
	GPIO.cleanup()








