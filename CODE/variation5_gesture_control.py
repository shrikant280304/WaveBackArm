"""
Variation 5: Gesture Based Servo Control
----------------------------------------
Uses open palm to move the servo and closed fist to reset to 0.
"""

import cv2
import mediapipe as mp
import numpy as np
import serial
import time

CAM_WIDTH, CAM_HEIGHT = 1280, 720
ARDUINO_PORT = "COM7"
BAUD_RATE = 9600

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

def is_open_palm(hand_landmarks):
    tips_ids = [4, 8, 12, 16, 20]
    for tip_id in tips_ids[1:]:  # Ignore thumb
        if hand_landmarks.landmark[tip_id].y > hand_landmarks.landmark[tip_id - 2].y:
            return False
    return True

while True:
    ret, img = cap.read()
    if not ret:
        break

    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = hands.process(imgRGB)

    if results.multi_hand_landmarks:
        landmarks = results.multi_hand_landmarks[0]

        if is_open_palm(landmarks):
            angle = detect_hand_angle(landmarks, CAM_WIDTH)
            send_servo_position(angle)
        else:
            send_servo_position(0)

    else:
        send_servo_position(0)

    cv2.imshow("Hand Tracking - Gesture Control", img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

arduino.close()
cap.release()
cv2.destroyAllWindows()
