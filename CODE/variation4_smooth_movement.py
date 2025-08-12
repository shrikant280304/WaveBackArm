"""
Variation 4: Smooth Servo Movement
----------------------------------
Adds smoothing to the servo movement to avoid jitter and abrupt changes.
"""

import cv2
import mediapipe as mp
import numpy as np
import serial
import time

CAM_WIDTH, CAM_HEIGHT = 1280, 720
ARDUINO_PORT = "COM7"
BAUD_RATE = 9600
SMOOTHING_FACTOR = 0.2

cap = cv2.VideoCapture(0)
cap.set(3, CAM_WIDTH)
cap.set(4, CAM_HEIGHT)

if not cap.isOpened():
    print("Camera couldn't be accessed!")
    exit()

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)

try:
    arduino = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
except serial.SerialException:
    print(f"Could not connect to Arduino on {ARDUINO_PORT}")
    exit()

def send_servo_position(angle: int):
    arduino.write(f"{angle}\n".encode())

def detect_hand_angle(hand_landmarks, width):
    wrist_x = int(hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].x * width)
    servo_angle = np.interp(wrist_x, [0, width], [0, 180])
    return int(servo_angle)

prev_angle = 0

while True:
    ret, img = cap.read()
    if not ret:
        break

    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = hands.process(imgRGB)

    if results.multi_hand_landmarks:
        angle = detect_hand_angle(results.multi_hand_landmarks[0], CAM_WIDTH)
        smooth_angle = int(prev_angle + SMOOTHING_FACTOR * (angle - prev_angle))
        send_servo_position(smooth_angle)
        prev_angle = smooth_angle
    else:
        send_servo_position(0)

    cv2.imshow("Hand Tracking - Smooth Movement", img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

arduino.close()
cap.release()
cv2.destroyAllWindows()
