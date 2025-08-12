"""
Variation 2: Dual Hand Tracking with Servo Control
---------------------------------------------------
Tracks two hands using OpenCV + MediaPipe, and sends
wrist position data to Arduino via PySerial to control two servos.
"""

import cv2
import mediapipe as mp
import numpy as np
import serial
import time

CAM_WIDTH, CAM_HEIGHT = 1280, 720
ARDUINO_PORT = "COM7"  # Change to your Arduino port
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
    max_num_hands=2,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)

try:
    arduino = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
except serial.SerialException:
    print(f"Could not connect to Arduino on {ARDUINO_PORT}")
    exit()

def send_servo_positions(angles):
    # Send both servo angles separated by a comma
    arduino.write(f"{angles[0]},{angles[1]}\n".encode())

def detect_hand_angle(hand_landmarks, width):
    wrist_x = int(hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].x * width)
    servo_angle = np.interp(wrist_x, [0, width], [0, 180])
    return int(servo_angle)

while True:
    ret, img = cap.read()
    if not ret:
        break

    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = hands.process(imgRGB)

    if results.multi_hand_landmarks:
        angles = []
        for hand_landmarks in results.multi_hand_landmarks:
            angles.append(detect_hand_angle(hand_landmarks, CAM_WIDTH))

        if len(angles) == 1:
            angles.append(0)  # If only one hand, second servo stays at 0

        send_servo_positions(angles)

        for hand_landmarks in results.multi_hand_landmarks:
            wrist_point = (
                int(hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].x * CAM_WIDTH),
                int(hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].y * CAM_HEIGHT)
            )
            cv2.circle(img, wrist_point, 10, (0, 0, 255), -1)

    else:
        send_servo_positions([0, 0])

    cv2.imshow("Hand Tracking - Dual", img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

arduino.close()
cap.release()
cv2.destroyAllWindows()
