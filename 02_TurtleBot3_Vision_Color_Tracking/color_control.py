#!/usr/bin/env python3
import sys
import argparse
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class ColorTeleopNode(Node):
    def __init__(self, topic, stamped, hsv_bounds, max_lin, max_ang):
        super().__init__('color_teleop_node')
        self.stamped = stamped
        self.max_lin = max_lin
        self.max_ang = max_ang
        self.lower_hsv = np.array(hsv_bounds[0], dtype=np.uint8)
        self.upper_hsv = np.array(hsv_bounds[1], dtype=np.uint8)

        if self.stamped:
            self.pub = self.create_publisher(TwistStamped, topic, 10)
        else:
            self.pub = self.create_publisher(Twist, topic, 10)

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Camera open করা যায়নি! /dev/video0 চেক করো।')
            sys.exit(1)

        # 30 FPS Timer loop
        self.timer = self.create_timer(0.033, self.process_frame)
        self.get_logger().info(f'Color Teleop চালু হয়েছে! Topic: {topic} (Stamped: {stamped})')

    def publish_cmd(self, lin_x, ang_z):
        if self.stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist.linear.x = float(lin_x)
            msg.twist.angular.z = float(ang_z)
            self.pub.publish(msg)
        else:
            msg = Twist()
            msg.linear.x = float(lin_x)
            msg.angular.z = float(ang_z)
            self.pub.publish(msg)

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        frame = cv2.flip(frame, 1)  # Mirror view
        h, w, _ = frame.shape
        center_x, center_y = w // 2, h // 2

        # Deadzone box (মাঝখানের নিরাপদ এলাকা যেখানে থাকলে রোবট থামবে)
        dead_w, dead_h = 100, 100
        box_x1, box_y1 = center_x - dead_w // 2, center_y - dead_h // 2
        box_x2, box_y2 = center_x + dead_w // 2, center_y + dead_h // 2

        # HSV Processing
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)
        
        # Noise reduction
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        lin_x = 0.0
        ang_z = 0.0
        status_text = "Object Not Found (Stopped)"

        if contours:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)

            if area > 500:  # Noise বাদ দেওয়ার জন্য ন্যূনতম ক্ষেত্রফল
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    # Draw tracked circle
                    cv2.circle(frame, (cx, cy), 12, (0, 255, 0), -1)

                    # Direction Logic
                    # Left / Right (Angular)
                    if cx < box_x1:
                        ang_z = self.max_ang   # বামে ঘুরবে
                    elif cx > box_x2:
                        ang_z = -self.max_ang  # ডানে ঘুরবে

                    # Up / Down (Linear)
                    if cy < box_y1:
                        lin_x = self.max_lin   # সামনে যাবে
                    elif cy > box_y2:
                        lin_x = -self.max_lin  # পেছনে যাবে

                    if lin_x == 0.0 and ang_z == 0.0:
                        status_text = "In Deadzone (Holding)"
                    else:
                        status_text = f"Moving: Lin={lin_x:.1f}, Ang={ang_z:.1f}"

        # Send command to ROS 2
        self.publish_cmd(lin_x, ang_z)

        # Draw UI
        cv2.rectangle(frame, (box_x1, box_y1), (box_x2, box_y2), (255, 255, 255), 2)
        cv2.putText(frame, status_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(frame, "Press 'q' to Quit", (20, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        cv2.imshow("Color Controller Feed", frame)
        cv2.imshow("Mask", mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.stop()

    def stop(self):
        self.publish_cmd(0.0, 0.0)
        self.cap.release()
        cv2.destroyAllWindows()
        rclpy.shutdown()

def main():
    parser = argparse.ArgumentParser(description="Color Controlled ROS 2 Teleop")
    parser.add_argument('--topic', default='/turtle1/cmd_vel', help="ROS cmd_vel topic")
    parser.add_argument('--stamped', action='store_true', help="Use TwistStamped instead of Twist")
    parser.add_argument('--color', default='red', choices=['red', 'green', 'blue'], help="Preset color")
    parser.add_argument('--hsv', nargs=6, type=int, help="Custom HSV: Hmin Smin Vmin Hmax Smax Vmax")
    parser.add_argument('--max-lin', type=float, default=1.5, help="Max linear speed")
    parser.add_argument('--max-ang', type=float, default=1.5, help="Max angular speed")
    args = parser.parse_args()

    # Color Presets (Default values)
    presets = {
        'red': ([0, 120, 70], [10, 255, 255]),
        'green': ([35, 80, 70], [85, 255, 255]),
        'blue': ([95, 120, 70], [130, 255, 255])
    }

    if args.hsv:
        hsv_bounds = (args.hsv[0:3], args.hsv[3:6])
    else:
        hsv_bounds = presets[args.color]

    rclpy.init()
    node = ColorTeleopNode(
        topic=args.topic,
        stamped=args.stamped,
        hsv_bounds=hsv_bounds,
        max_lin=args.max_lin,
        max_ang=args.max_ang
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.stop()

if __name__ == '__main__':
    main()
