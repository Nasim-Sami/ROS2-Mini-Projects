#!/usr/bin/env python3
import cv2
import numpy as np

def nothing(x):
    pass

cv2.namedWindow('HSV Tuner')
# Initial values for RED color detection
cv2.createTrackbar('H Min', 'HSV Tuner', 0, 179, nothing)
cv2.createTrackbar('H Max', 'HSV Tuner', 10, 179, nothing)
cv2.createTrackbar('S Min', 'HSV Tuner', 120, 255, nothing)
cv2.createTrackbar('S Max', 'HSV Tuner', 255, 255, nothing)
cv2.createTrackbar('V Min', 'HSV Tuner', 70, 255, nothing)
cv2.createTrackbar('V Max', 'HSV Tuner', 255, 255, nothing)

cap = cv2.VideoCapture(0)

print("\n--- HSV Tuner চালু হয়েছে ---")
print("১. স্লাইডার নেড়ে শুধু তোমার রঙিন অবজেক্টটিকে সাদা (White) করো।")
print("২. টার্মিনালে রেঞ্জ প্রিন্ট করতে কিবোর্ডে 'p' চাপো।")
print("৩. বের হতে 'q' বা Esc চাপো।\n")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)  # Mirror view
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    h_min = cv2.getTrackbarPos('H Min', 'HSV Tuner')
    h_max = cv2.getTrackbarPos('H Max', 'HSV Tuner')
    s_min = cv2.getTrackbarPos('S Min', 'HSV Tuner')
    s_max = cv2.getTrackbarPos('S Max', 'HSV Tuner')
    v_min = cv2.getTrackbarPos('V Min', 'HSV Tuner')
    v_max = cv2.getTrackbarPos('V Max', 'HSV Tuner')

    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])

    mask = cv2.inRange(hsv, lower, upper)
    result = cv2.bitwise_and(frame, frame, mask=mask)

    cv2.imshow('Original Camera', frame)
    cv2.imshow('Mask (Only object should be white)', mask)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('p'):
        print(f"\n[HSV Values] --hsv {h_min} {s_min} {v_min} {h_max} {s_max} {v_max}")
    elif key == ord('q') or key == 27:
        break

cap.release()
cv2.destroyAllWindows()
