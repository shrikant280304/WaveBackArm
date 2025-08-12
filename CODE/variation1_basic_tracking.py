"""
Variation 1: Basic Hand Tracking with Servo Control
---------------------------------------------------
Tracks a single hand using OpenCV + MediaPipe, and sends
wrist position data to Arduino via PySerial to control a servo.
"""

import cv2
import mediapipe as mp
import numpy as np
import serial
import time

# ------------------- CONFIGURATION -------------------
CAM_WIDTH, CAM_HEIGHT = 1280, 720
ARDUINO_PORT = "COM7"  # Change to your Arduino port
BAUD_RATE = 9600
SERVO_MIN_ANGLE = 0
SERVO_MAX_ANGLE = 180
# ------------------------------------------------------

# Initialize webcam
cap = cv2.VideoCapture(0)
cap.set(3, CAM_WIDTH)
cap.set(4, CAM_HEIGHT)

if not cap.isOpened():
    print("Camera couldn't be accessed!")
    exit()

# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)

# Open serial connection to Arduino
try:
    arduino = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # Wait for Arduino to initialize
except serial.SerialException:
    print(f"Could not connect to Arduino on {ARDUINO_PORT}")
    exit()

def send_servo_position(angle: int):
    """Send servo angle to Arduino via serial."""
    arduino.write(f"{angle}\n".encode())

def detect_hand_angle(hand_landmarks, width, height) -> float:
    """Convert wrist position to servo angle."""
    wrist = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST]
    wrist_x = int(wrist.x * width)
    servo_angle = np.interp(wrist_x, [0, width], [SERVO_MIN_ANGLE, SERVO_MAX_ANGLE])
    return servo_angle

while True:
    ret, img = cap.read()
    if not ret:
        break

    # Convert frame to RGB
    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # Process with MediaPipe
    results = hands.process(imgRGB)

    if results.multi_hand_landmarks:
        hand_landmarks = results.multi_hand_landmarks[0]

        # Calculate servo angle
        servo_angle = detect_hand_angle(hand_landmarks, CAM_WIDTH, CAM_HEIGHT)

        # Send to Arduino
        send_servo_position(int(servo_angle))

        # Draw visual indicators
        wrist_point = (
            int(hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].x * CAM_WIDTH),
            int(hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].y * CAM_HEIGHT)
        )
        cv2.circle(img, wrist_point, 10, (0, 0, 255), -1)
        cv2.putText(img, f'Servo: {int(servo_angle)} deg', (50, 50),
                    cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)

    else:
        # No hands detected
        send_servo_position(0)
        cv2.putText(img, "NO HAND DETECTED", (50, 50),
                    cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2)

    # Show video
    cv2.imshow("Hand Tracking - Basic", img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
arduino.close()
cap.release()
cv2.destroyAllWindows()
