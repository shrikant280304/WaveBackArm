# Hand Tracking Servo Control with Python & Arduino

This project uses **OpenCV** and **MediaPipe** to track hand movement in real-time 
and control a servo motor via Arduino. The servo "waves back" to the user.

## 📜 Variations
1. **Basic Tracking (Current)** – Tracks one hand and moves a single servo.

*(Future variations can include dual-hand tracking, angle limits, smoothing, and gesture-based control.)*

## 🛠 Hardware
- Arduino Uno (or compatible)
- Servo Motor (SG90/MG995)
- USB Webcam / Laptop Camera
- USB Cable

## 💻 Software
- Python 3.8+
- Arduino IDE

## 📦 Installation
```bash
git clone https://github.com/yourusername/hand-wave-servo.git
cd hand-wave-servo
pip install -r requirements.txt
```

## ▶️ Running the Code
1. Connect Arduino and upload the servo control sketch from `arduino/servo_controller.ino`.
2. Update `ARDUINO_PORT` in the script to your correct COM port.
3. Run the variation:
```bash
python variations/variation1_basic_tracking.py
```
4. Press `q` to quit.

## 📹 Demo
(https://youtu.be/Dy3oX6qcPuQ)

## 📄 License
MIT License
