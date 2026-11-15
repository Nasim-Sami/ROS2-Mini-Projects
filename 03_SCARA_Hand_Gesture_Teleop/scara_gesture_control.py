#!/usr/bin/env python3
import os
import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

class ScaraPerfectGestureTeleop(Node):
    def __init__(self):
        super().__init__('scara_perfect_gesture_teleop')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        # MediaPipe Tasks HandLandmarker Setup
        model_path = os.path.expanduser('~/hand_landmarker.task')
        base_options = python.BaseOptions(model_asset_path=model_path)
        options = vision.HandLandmarkerOptions(
            base_options=base_options,
            running_mode=vision.RunningMode.IMAGE,
            num_hands=1,
            min_hand_detection_confidence=0.55
        )
        self.detector = vision.HandLandmarker.create_from_options(options)

        self.cap = cv2.VideoCapture(0)
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        
        # Smoothed joint values
        self.j1 = 0.0
        self.j2 = 0.0
        self.j3 = 0.0
        self.j4 = 0.0

        # Balanced smooth speed
        self.alpha = 0.16

        self.timer = self.create_timer(0.033, self.loop)
        self.get_logger().info('SCARA Perfect Gesture Teleop Active!')

    def clamp(self, val, min_val, max_val):
        return max(min_val, min(max_val, val))

    def loop(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        frame = cv2.flip(frame, 1)  # Mirror view
        h, w, _ = frame.shape
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        result = self.detector.detect(mp_image)

        status_text = "IDLE (HOLDING)"
        status_color = (180, 180, 180)

        # Draw Center Crosshair / Neutral Box
        cx, cy = w // 2, h // 2
        cv2.rectangle(frame, (cx - 40, cy - 40), (cx + 40, cy + 40), (80, 80, 80), 1)

        if result.hand_landmarks:
            lms = result.hand_landmarks[0]

            wrist = lms[0]
            thumb_tip = lms[4]
            index_tip = lms[8]
            middle_mcp = lms[9]
            middle_tip = lms[12]
            ring_tip = lms[16]
            pinky_tip = lms[20]

            # Hand palm length reference (Scale invariant)
            palm_len = math.hypot(middle_mcp.x - wrist.x, middle_mcp.y - wrist.y)
            if palm_len < 0.01:
                palm_len = 0.01

            # 1. ROBUST FIST DETECTION (মুষ্টি চেনার নির্ভরযোগ্য নিয়ম)
            d8 = math.hypot(index_tip.x - wrist.x, index_tip.y - wrist.y)
            d12 = math.hypot(middle_tip.x - wrist.x, middle_tip.y - wrist.y)
            d16 = math.hypot(ring_tip.x - wrist.x, ring_tip.y - wrist.y)
            d20 = math.hypot(pinky_tip.x - wrist.x, pinky_tip.y - wrist.y)
            avg_tip_dist = (d8 + d12 + d16 + d20) / 4.0
            fist_ratio = avg_tip_dist / palm_len
            is_fist = fist_ratio < 1.08  # All fingers tightly curled

            # 2. ROBUST PINCH DETECTION (চিমটি চেনার নিয়ম)
            pinch_dist = math.hypot(thumb_tip.x - index_tip.x, thumb_tip.y - index_tip.y)
            is_pinch = (pinch_dist / palm_len) < 0.28 and not is_fist

            # Hand offset from center
            dx = wrist.x - 0.5
            dy = wrist.y - 0.5

            # ================= GESTURE CLASSIFICATION =================

            # CASE A: FIST ACTIVE (মুষ্টি করা হয়েছে)
            if is_fist:
                # Measure fist tilt: vector from wrist to middle knuckle
                f_dx = middle_mcp.x - wrist.x
                f_dy = middle_mcp.y - wrist.y
                fist_tilt = math.atan2(f_dx, -f_dy)

                # A1: মুষ্টি ডানে বা বামে কাত করলে -> Joint 4 (Gripper 360)
                if abs(fist_tilt) > 0.22:
                    status_text = ">> FIST TILTED -> JOINT 4 (GRIPPER 360)"
                    status_color = (255, 0, 255)  # Magenta
                    target_j4 = self.clamp(fist_tilt * 2.8, -3.14, 3.14)
                    self.j4 = self.alpha * target_j4 + (1 - self.alpha) * self.j4
                
                # A2: মুষ্টি সোজা রেখে উপরে-নিচে নিলে -> Joint 3 (Elevator)
                else:
                    status_text = ">> FIST UPRIGHT -> JOINT 3 (Z-AXIS UP/DOWN)"
                    status_color = (0, 255, 255)  # Yellow
                    norm_y = np.clip((wrist.y - 0.25) / 0.45, 0.0, 1.0)
                    target_j3 = 0.03 - (norm_y * 0.19)
                    self.j3 = self.alpha * target_j3 + (1 - self.alpha) * self.j3

                # Arm joints J1 & J2 stay completely LOCKED during fist!

            # CASE B: PINCH ACTIVE (চিমটি কাটা হয়েছে)
            elif is_pinch:
                status_text = ">> PINCH ACTIVE -> JOINT 3 (PICK DOWN)"
                status_color = (0, 200, 255)  # Cyan
                target_j3 = -0.15  # Plunge down
                self.j3 = self.alpha * target_j3 + (1 - self.alpha) * self.j3
                # J1, J2, J4 locked!

            # CASE C: OPEN HAND (হাত খোলা রেখে আর্ম পজিশন করা)
            else:
                abs_dx = abs(dx)
                abs_dy = abs(dy)

                # C1: পরিষ্কারভাবে ডানে-বামে যাওয়া (Dominant Horizontal)
                if abs_dx > (abs_dy * 1.25) and abs_dx > 0.06:
                    status_text = ">> OPEN HAND -> JOINT 1 (SHOULDER)"
                    status_color = (0, 255, 0)  # Green
                    target_j1 = self.clamp(dx * 2.8, -2.8, 2.8)
                    self.j1 = self.alpha * target_j1 + (1 - self.alpha) * self.j1

                # C2: পরিষ্কারভাবে উপরে-নিচে যাওয়া (Dominant Vertical)
                elif abs_dy > (abs_dx * 1.25) and abs_dy > 0.06:
                    status_text = ">> OPEN HAND -> JOINT 2 (ELBOW)"
                    status_color = (255, 140, 0)  # Orange
                    target_j2 = self.clamp(-dy * 2.2, -2.0, 2.0)
                    self.j2 = self.alpha * target_j2 + (1 - self.alpha) * self.j2

                else:
                    status_text = "OPEN HAND (HOLDING CENTER)"
                    status_color = (200, 200, 200)

            # Draw Hand Visuals
            cv2.circle(frame, (int(wrist.x * w), int(wrist.y * h)), 8, status_color, -1)
            if is_fist:
                # Draw fist tilt line
                p_knuckle = (int(middle_mcp.x * w), int(middle_mcp.y * h))
                p_wrist = (int(wrist.x * w), int(wrist.y * h))
                cv2.line(frame, p_wrist, p_knuckle, (255, 0, 255), 3)

        # Publish JointState to ROS 2
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [float(self.j1), float(self.j2), float(self.j3), float(self.j4)]
        self.pub.publish(msg)

        # UI Overlay
        cv2.putText(frame, status_text, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.65, status_color, 2)
        cv2.putText(frame, f"J1(Shoulder): {self.j1:+.2f} rad", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        cv2.putText(frame, f"J2(Elbow)   : {self.j2:+.2f} rad", (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        cv2.putText(frame, f"J3(Z-Shaft) : {self.j3:+.2f} m",   (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        cv2.putText(frame, f"J4(Gripper) : {self.j4:+.2f} rad", (20, 145), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

        cv2.imshow("SCARA Perfect Teleop", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.stop()

    def stop(self):
        self.cap.release()
        cv2.destroyAllWindows()
        rclpy.shutdown()

def main():
    rclpy.init()
    node = ScaraPerfectGestureTeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.stop()

if __name__ == '__main__':
    main()
