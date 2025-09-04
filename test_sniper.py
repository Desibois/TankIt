import cv2
import mediapipe as mp
from adafruit_servokit import ServoKit
from collections import deque
import RPi.GPIO as GPIO
import time


mode = 'auto'
# ---------------- Relay / Gun Setup ----------------
relay_pin = 14  # BCM numbering
GPIO.setmode(GPIO.BCM)
GPIO.setup(relay_pin, GPIO.OUT)
GPIO.output(relay_pin, GPIO.LOW)  # Ensure off at start

# ---------------- Servo Setup ----------------
kit = ServoKit(channels=16)
kit.servo[0].set_pulse_width_range(500, 2500)  # PAN only

# ---------------- Camera Setup ----------------
cap = cv2.VideoCapture(0)

# Initial pan angle
pan_angle = 90
kit.servo[0].angle = pan_angle

# Physical limits
PAN_MIN, PAN_MAX = 0, 180

# Tracking stability settings
ANGLE_THRESHOLD = 2.0
SMOOTHING = 0.8
HISTORY_LEN = 5

# Camera FOV
HFOV = 62

# Targeting settings
TARGET_TOLERANCE_DEG = 1.5  # degrees from horizontal center to "lock on"
FIRE_COOLDOWN = 1.0         # seconds between shots

# Moving average buffer for horizontal movement
cx_history = deque(maxlen=HISTORY_LEN)

# Cooldown timer
last_fire_time = 0

# Mediapipe setup
mp_face_detection = mp.solutions.face_detection
face_detection = mp_face_detection.FaceDetection(model_selection=0, min_detection_confidence=0.6)

def camera_feed(frame, detected):
	if detected:
		cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
		cv2.circle(frame, (int(avg_cx), y + h // 2), 5, (0, 0, 255), -1)
		cv2.putText(frame, f"Pan: {int(pan_angle)}°", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
	else:
		cv2.putText(frame, "No face detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

	cv2.imshow("Face Tracking", frame)


def fire():
	global last_fire_time
	if time.time() - last_fire_time > FIRE_COOLDOWN:
		print("TARGET LOCKED - FIRING")
		GPIO.output(relay_pin, GPIO.HIGH)  # Turn on relay
		time.sleep(0.5)                    # Fire duration
		GPIO.output(relay_pin, GPIO.LOW)   # Turn off relay
		last_fire_time = time.time()
	
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Camera read failed")
            continue

        height, width, _ = frame.shape
        cx_center = width // 2

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = face_detection.process(rgb_frame)

        if results.detections:
            detection = results.detections[0]
            box = detection.location_data.relative_bounding_box

            # Pixel coordinates
            x = int(box.xmin * width)
            y = int(box.ymin * height)
            w = int(box.width * width)
            h = int(box.height * height)

            # Face center (horizontal only for tracking)
            cx = x + w // 2

            # Update moving average (horizontal only)
            cx_history.append(cx)
            avg_cx = sum(cx_history) / len(cx_history)

            # Pixel offset from horizontal center
            dx = avg_cx - cx_center

            # Convert to angular offset
            offset_pan = (dx / width) * HFOV

            # Calculate new pan angle
            new_pan = pan_angle - offset_pan

            # Dead zone + smoothing for pan only
            if abs(new_pan - pan_angle) > ANGLE_THRESHOLD:
                pan_angle = (1 - SMOOTHING) * new_pan + SMOOTHING * pan_angle
                pan_angle = max(PAN_MIN, min(PAN_MAX, pan_angle))
                kit.servo[0].angle = pan_angle

            # Fire if horizontally centered (ignore vertical position)
            if abs(offset_pan) < TARGET_TOLERANCE_DEG:
                fire()
                
        camera_feed(frame, results.detections)
                    
        if cv2.waitKey(1) & 0xFF == 27:  # ESC
            break

finally:
    cap.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()
