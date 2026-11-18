#!/usr/bin/env python3
"""
Autonomous Drone YOLOv8 Vision & Obstacle Perception Node
---------------------------------------------------------
Subscribes to: /camera/image_raw (sensor_msgs/msg/Image)
Detects: Persons, Vehicles, Objects via Ultralytics YOLOv8
Computes: Real-time proximity, obstacle sector, collision threat level
Publishes:
- /drone/vision_perception (std_msgs/msg/String: JSON perception summary)
- /camera/annotated_image (sensor_msgs/msg/Image: with bounding boxes)
Saves:
- Snapshots to /workspace/captures/ on photo triggers
"""

import sys
import os
import json
import time
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from ultralytics import YOLO

CAPTURES_DIR = "/workspace/captures"
os.makedirs(CAPTURES_DIR, exist_ok=True)


class DroneVisionDetector(Node):
    def __init__(self):
        super().__init__('drone_vision_detector')

        # Load lightweight YOLOv8 nano model
        self.get_logger().info("🔍 Loading YOLOv8 nano neural network...")
        self.model = YOLO('yolov8n.pt')
        self.get_logger().info("✅ YOLOv8 model loaded successfully.")

        self.frame_count = 0
        self.last_detection_summary = {
            "obstacle_ahead": False,
            "obstacle_type": None,
            "confidence": 0.0,
            "estimated_dist_m": None,
            "detected_objects": []
        }

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.photo_trigger_sub = self.create_subscription(
            String,
            '/drone/trigger_photo',
            self.photo_trigger_callback,
            10
        )

        # Publishers
        self.perception_pub = self.create_publisher(
            String,
            '/drone/vision_perception',
            10
        )
        self.annotated_image_pub = self.create_publisher(
            Image,
            '/camera/annotated_image',
            10
        )

        self.latest_annotated_frame = None
        self.get_logger().info("👁️ Drone Vision Detector is ACTIVE and monitoring /camera/image_raw")

    def image_callback(self, msg: Image):
        self.frame_count += 1
        # Run inference every 3rd frame (~10Hz)
        if self.frame_count % 3 != 0:
            return

        try:
            raw = np.frombuffer(msg.data, dtype=np.uint8)
            if msg.encoding in ['rgb8', 'RGB8']:
                cv_img = cv2.cvtColor(raw.reshape((msg.height, msg.width, 3)), cv2.COLOR_RGB2BGR)
            elif msg.encoding in ['bgr8', 'BGR8']:
                cv_img = raw.reshape((msg.height, msg.width, 3))
            else:
                cv_img = raw.reshape((msg.height, msg.width, -1))
        except Exception as e:
            self.get_logger().warn(f"Image unpack warning: {e}")
            return

        h, w, _ = cv_img.shape

        # Run YOLO inference
        results = self.model(cv_img, verbose=False, conf=0.35)

        detected_list = []
        obstacle_ahead = False
        obstacle_type = None
        obstacle_conf = 0.0
        min_estimated_dist = 999.0

        for r in results:
            boxes = r.boxes
            for box in boxes:
                cls_id = int(box.cls[0])
                cls_name = self.model.names[cls_id]
                conf = float(box.conf[0])
                xyxy = box.xyxy[0].cpu().numpy()

                x1, y1, x2, y2 = xyxy
                box_w = x2 - x1
                box_h = y2 - y1
                area_ratio = (box_w * box_h) / (w * h)
                center_x = (x1 + x2) / 2.0

                # Estimated rough distance based on apparent pixel height in 1280x960 frame
                # Standard human (~1.7m) at 3m distance spans ~35% of frame height
                apparent_h_ratio = box_h / float(h)
                if apparent_h_ratio > 0.05:
                    est_dist = round(max(0.5, 1.2 / apparent_h_ratio), 1)
                else:
                    est_dist = 10.0

                detected_list.append({
                    "class": cls_name,
                    "confidence": round(conf, 2),
                    "box": [int(x1), int(y1), int(box_w), int(box_h)],
                    "est_dist_m": est_dist
                })

                # Obstacle check: Center field of view (|x - w/2| < w/3) and distance < 4.0m
                is_centered = abs(center_x - (w / 2.0)) < (w / 3.0)
                if cls_name in ["person", "car", "truck", "bus", "motorcycle", "box"] and is_centered:
                    if est_dist < 4.0 and est_dist < min_estimated_dist:
                        obstacle_ahead = True
                        obstacle_type = cls_name
                        obstacle_conf = conf
                        min_estimated_dist = est_dist

        # Draw annotations
        annotated_frame = results[0].plot()

        # Add HUD overlay
        if obstacle_ahead:
            hud_text = f"WARNING: {obstacle_type.upper()} AHEAD (~{min_estimated_dist:.1f}m)"
            cv2.putText(annotated_frame, hud_text, (40, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
            # Red border
            cv2.rectangle(annotated_frame, (0, 0), (w-1, h-1), (0, 0, 255), 6)
        else:
            cv2.putText(annotated_frame, "PATH CLEAR", (40, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

        self.latest_annotated_frame = annotated_frame

        # Publish annotated ROS image
        try:
            ros_annotated = Image()
            ros_annotated.header = msg.header
            ros_annotated.height = h
            ros_annotated.width = w
            ros_annotated.encoding = 'bgr8'
            ros_annotated.step = w * 3
            ros_annotated.data = annotated_frame.tobytes()
            self.annotated_image_pub.publish(ros_annotated)
        except Exception:
            pass

        # Publish JSON perception summary
        summary = {
            "obstacle_ahead": bool(obstacle_ahead),
            "obstacle_type": str(obstacle_type) if obstacle_type else None,
            "confidence": round(float(obstacle_conf), 2),
            "estimated_dist_m": round(float(min_estimated_dist), 1) if obstacle_ahead else None,
            "detected_objects": detected_list,
            "timestamp": float(time.time())
        }
        self.last_detection_summary = summary

        sum_msg = String()
        sum_msg.data = json.dumps(summary, default=float)
        self.perception_pub.publish(sum_msg)

        if obstacle_ahead and self.frame_count % 30 == 0:
            self.get_logger().warn(f"🚨 Obstacle Detected: '{obstacle_type}' at ~{min_estimated_dist:.1f}m ahead!")

    def photo_trigger_callback(self, msg: String):
        label = msg.data or "manual_capture"
        if self.latest_annotated_frame is not None:
            filename = f"capture_{int(time.time())}_{label}.jpg"
            filepath = os.path.join(CAPTURES_DIR, filename)
            cv2.imwrite(filepath, self.latest_annotated_frame)
            self.get_logger().info(f"📸 Aerial Snapshot Saved: {filepath}")
            # Also save latest.jpg for easy web viewing
            latest_path = os.path.join(CAPTURES_DIR, "latest.jpg")
            cv2.imwrite(latest_path, self.latest_annotated_frame)


def main(args=None):
    rclpy.init(args=args)
    node = DroneVisionDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
