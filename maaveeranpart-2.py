import cv2
import serial
import time
import mediapipe as mp
import math

# Set up serial communication
arduino = serial.Serial('COM5', 9600, timeout=1)
time.sleep(2)

# MediaPipe setup
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()
mp_drawing = mp.solutions.drawing_utils

# Capture camera
cap = cv2.VideoCapture(r'C:\Users\RITS\Pictures\maa.mp4')

def calculate_angle(a, b, c):
    # Calculate angle between three points (in degrees)
    ba = a - b
    bc = c - b

    angle = math.degrees(
        math.acos(
            (ba @ bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
        )
    )
    return int(angle)

import numpy as np

def get_joint_angle(landmarks, p1, p2, p3):
    a = np.array([landmarks[p1].x, landmarks[p1].y])
    b = np.array([landmarks[p2].x, landmarks[p2].y])
    c = np.array([landmarks[p3].x, landmarks[p3].y])
    return calculate_angle(a, b, c)

def send_angles(left, right):
    msg = f"{left},{right}\n"
    arduino.write(msg.encode())
    print(f"[Serial] Sent: {msg.strip()}")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = pose.process(rgb)

    if results.pose_landmarks:
        lm = results.pose_landmarks.landmark

        try:
            left_angle = get_joint_angle(lm, mp_pose.PoseLandmark.LEFT_SHOULDER,
                                              mp_pose.PoseLandmark.LEFT_ELBOW,
                                              mp_pose.PoseLandmark.LEFT_WRIST)

            right_angle = get_joint_angle(lm, mp_pose.PoseLandmark.RIGHT_SHOULDER,
                                               mp_pose.PoseLandmark.RIGHT_ELBOW,
                                               mp_pose.PoseLandmark.RIGHT_WRIST)

            left_angle = 180 - left_angle  # Mirror correction if needed
            right_angle = right_angle

            send_angles(left_angle, right_angle)

        except Exception as e:
            print("Angle calc error:", e)

        mp_drawing.draw_landmarks(frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

    cv2.imshow("Pose Tracking", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

send_angles(90, 90)  # Reset before closing
cap.release()
cv2.destroyAllWindows()
arduino.close()
