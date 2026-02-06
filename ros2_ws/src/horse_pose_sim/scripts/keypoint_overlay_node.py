#!/usr/bin/env python3

from typing import List, Optional

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from horse_pose_sim.msg import HorseDetections


class KeypointOverlayNode(Node):
    def __init__(self) -> None:
        super().__init__("keypoint_overlay_node")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("detections_topic", "/horse_keypoints/detections")
        self.declare_parameter("overlay_topic", "/horse_keypoints/overlay_from_msg")
        self.declare_parameter("max_age_sec", 0.30)

        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.detections_topic = self.get_parameter("detections_topic").get_parameter_value().string_value
        self.overlay_topic = self.get_parameter("overlay_topic").get_parameter_value().string_value
        self.max_age_sec = self.get_parameter("max_age_sec").get_parameter_value().double_value

        self.bridge = CvBridge()
        self.latest_detections: Optional[HorseDetections] = None

        self.det_sub = self.create_subscription(HorseDetections, self.detections_topic, self.det_callback, 10)
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.overlay_pub = self.create_publisher(Image, self.overlay_topic, 10)

        self.get_logger().info(
            f"keypoint_overlay_node ready. image_topic={self.image_topic}, detections_topic={self.detections_topic}, overlay_topic={self.overlay_topic}"
        )

    def det_callback(self, msg: HorseDetections) -> None:
        self.latest_detections = msg

    def image_callback(self, image_msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"cv_bridge conversion failed in overlay node: {exc}")
            return

        detections = self.latest_detections
        if detections is not None:
            det_time = detections.header.stamp.sec + detections.header.stamp.nanosec * 1e-9
            img_time = image_msg.header.stamp.sec + image_msg.header.stamp.nanosec * 1e-9
            if abs(img_time - det_time) > self.max_age_sec:
                detections = None

        if detections is not None:
            self.draw_detections(frame, detections)
        else:
            cv2.putText(
                frame,
                "Waiting for synchronized detections...",
                (18, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 200, 255),
                2,
                cv2.LINE_AA,
            )

        self.overlay_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding="bgr8"))

    def draw_detections(self, frame, detections: HorseDetections) -> None:
        for det in detections.detections:
            cx = det.bbox_center_x
            cy = det.bbox_center_y
            w = det.bbox_width
            h = det.bbox_height

            x1 = int(cx - w / 2.0)
            y1 = int(cy - h / 2.0)
            x2 = int(cx + w / 2.0)
            y2 = int(cy + h / 2.0)

            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 180, 0), 2)
            cv2.putText(
                frame,
                f"msg horse#{det.track_id} {det.score:.2f}",
                (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 180, 0),
                2,
                cv2.LINE_AA,
            )

            for kp in det.keypoints:
                if kp.confidence < 0.20:
                    continue
                pt = (int(kp.x), int(kp.y))
                cv2.circle(frame, pt, 3, (0, 255, 255), -1)


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = KeypointOverlayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
