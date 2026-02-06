#!/usr/bin/env python3

import os
import time
from typing import List

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from horse_pose_sim.msg import HorseDetection, HorseDetections, HorseKeypoint


class HorsePoseNode(Node):
    def __init__(self) -> None:
        super().__init__("horse_pose_node")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("detections_topic", "/horse_keypoints/detections")
        self.declare_parameter("annotated_image_topic", "/horse_keypoints/annotated")
        self.declare_parameter("model_path", "")
        self.declare_parameter("confidence_threshold", 0.35)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("publish_annotated_image", True)
        self.declare_parameter("debug_save_dir", "")
        self.declare_parameter("debug_save_every_n", 60)

        self.image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self.detections_topic = (
            self.get_parameter("detections_topic").get_parameter_value().string_value
        )
        self.annotated_topic = (
            self.get_parameter("annotated_image_topic")
            .get_parameter_value()
            .string_value
        )
        self.model_path = (
            self.get_parameter("model_path").get_parameter_value().string_value
        )
        self.conf_thresh = (
            self.get_parameter("confidence_threshold")
            .get_parameter_value()
            .double_value
        )
        self.device = self.get_parameter("device").get_parameter_value().string_value
        self.publish_annotated = (
            self.get_parameter("publish_annotated_image")
            .get_parameter_value()
            .bool_value
        )
        self.debug_save_dir = (
            self.get_parameter("debug_save_dir").get_parameter_value().string_value
        )
        self.debug_save_every_n = (
            self.get_parameter("debug_save_every_n").get_parameter_value().integer_value
        )

        if not self.model_path:
            raise RuntimeError(
                "Parameter 'model_path' is empty. Provide path to YOLO pose model .pt file."
            )
        if not os.path.exists(self.model_path):
            raise RuntimeError(f"YOLO model path not found: {self.model_path}")

        try:
            from ultralytics import YOLO
        except Exception as exc:
            raise RuntimeError(
                "Failed to import ultralytics. Install dependencies with scripts/install_python_deps.sh"
            ) from exc

        self.model = YOLO(self.model_path)
        self.model_name = os.path.basename(self.model_path)

        self.bridge = CvBridge()
        self.frame_counter = 0

        if self.debug_save_dir:
            os.makedirs(self.debug_save_dir, exist_ok=True)

        self.detections_pub = self.create_publisher(
            HorseDetections, self.detections_topic, 10
        )
        self.annotated_pub = self.create_publisher(Image, self.annotated_topic, 10)

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"horse_pose_node ready. image_topic={self.image_topic}, detections_topic={self.detections_topic}, model={self.model_name}"
        )

    def image_callback(self, msg: Image) -> None:
        self.frame_counter += 1
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"cv_bridge conversion failed: {exc}")
            return

        start = time.time()
        try:
            result = self.model.predict(
                source=frame,
                conf=float(self.conf_thresh),
                device=self.device,
                verbose=False,
            )[0]
        except Exception as exc:
            self.get_logger().error(f"YOLO inference failed: {exc}")
            return

        detections_msg, annotated = self._build_outputs(msg, frame, result)
        self.detections_pub.publish(detections_msg)

        if self.publish_annotated:
            self.annotated_pub.publish(
                self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            )

        if (
            self.debug_save_dir
            and self.debug_save_every_n > 0
            and self.frame_counter % self.debug_save_every_n == 0
        ):
            frame_path = os.path.join(
                self.debug_save_dir, f"camera_raw_{self.frame_counter:06d}.png"
            )
            annotated_path = os.path.join(
                self.debug_save_dir, f"camera_annotated_{self.frame_counter:06d}.png"
            )
            cv2.imwrite(frame_path, frame)
            cv2.imwrite(annotated_path, annotated)

        latency_ms = (time.time() - start) * 1000.0
        self.get_logger().debug(
            f"Processed frame={self.frame_counter} detections={len(detections_msg.detections)} latency_ms={latency_ms:.1f}"
        )

    def _build_outputs(
        self, image_msg: Image, frame: np.ndarray, result
    ) -> tuple[HorseDetections, np.ndarray]:
        annotated = frame.copy()

        detections_msg = HorseDetections()
        detections_msg.header = image_msg.header
        detections_msg.model_name = self.model_name

        if result.boxes is None or len(result.boxes) == 0:
            cv2.putText(
                annotated,
                "No horse detections",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
            return detections_msg, annotated

        xywh = (
            result.boxes.xywh.cpu().numpy()
            if result.boxes.xywh is not None
            else np.zeros((0, 4))
        )
        scores = (
            result.boxes.conf.cpu().numpy()
            if result.boxes.conf is not None
            else np.zeros((0,))
        )

        keypoints_xy = None
        keypoints_conf = None
        if result.keypoints is not None:
            if result.keypoints.xy is not None:
                keypoints_xy = result.keypoints.xy.cpu().numpy()
            if result.keypoints.conf is not None:
                keypoints_conf = result.keypoints.conf.cpu().numpy()

        for det_idx in range(len(xywh)):
            det_msg = HorseDetection()
            det_msg.track_id = int(det_idx)
            det_msg.score = float(scores[det_idx]) if det_idx < len(scores) else 0.0
            det_msg.bbox_center_x = float(xywh[det_idx][0])
            det_msg.bbox_center_y = float(xywh[det_idx][1])
            det_msg.bbox_width = float(xywh[det_idx][2])
            det_msg.bbox_height = float(xywh[det_idx][3])

            cx, cy, w, h = xywh[det_idx]
            x1 = int(cx - w / 2.0)
            y1 = int(cy - h / 2.0)
            x2 = int(cx + w / 2.0)
            y2 = int(cy + h / 2.0)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (40, 220, 40), 2)
            cv2.putText(
                annotated,
                f"horse#{det_idx} {det_msg.score:.2f}",
                (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (20, 220, 20),
                2,
                cv2.LINE_AA,
            )

            if keypoints_xy is not None and det_idx < len(keypoints_xy):
                kp_xy = keypoints_xy[det_idx]
                kp_conf_row = None
                if keypoints_conf is not None and det_idx < len(keypoints_conf):
                    kp_conf_row = keypoints_conf[det_idx]

                for kp_idx in range(len(kp_xy)):
                    conf = 1.0
                    if kp_conf_row is not None and kp_idx < len(kp_conf_row):
                        conf = float(kp_conf_row[kp_idx])

                    kp_msg = HorseKeypoint()
                    kp_msg.name = f"kp_{kp_idx}"
                    kp_msg.x = float(kp_xy[kp_idx][0])
                    kp_msg.y = float(kp_xy[kp_idx][1])
                    kp_msg.confidence = conf
                    det_msg.keypoints.append(kp_msg)

                    if conf >= 0.20:
                        pt = (int(kp_msg.x), int(kp_msg.y))
                        cv2.circle(annotated, pt, 4, (0, 140, 255), -1)
                        cv2.putText(
                            annotated,
                            str(kp_idx),
                            (pt[0] + 5, pt[1] - 4),
                            cv2.FONT_HERSHEY_PLAIN,
                            0.9,
                            (255, 255, 255),
                            1,
                            cv2.LINE_AA,
                        )

            detections_msg.detections.append(det_msg)

        return detections_msg, annotated


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = HorsePoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
