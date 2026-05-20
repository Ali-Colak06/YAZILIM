import json
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String


class ArucoDetectionDemo(Node):
    def __init__(self):
        super().__init__("aruco_detection_demo_node")

        self.declare_parameter("image_topic", "/realsense/rgb/image_raw")
        self.declare_parameter("debug_image_topic", "/aruco/debug_image")
        self.declare_parameter("detections_topic", "/aruco/detections")
        self.declare_parameter("aruco_dictionary", "DICT_5X5_250")
        self.declare_parameter("min_marker_id", -1)
        self.declare_parameter("max_marker_id", -1)
        self.declare_parameter("publish_detections", True)
        self.declare_parameter("draw_rejected", False)
        self.declare_parameter("log_interval_sec", 1.0)

        self.image_topic = self.get_parameter("image_topic").value
        self.debug_image_topic = self.get_parameter("debug_image_topic").value
        self.detections_topic = self.get_parameter("detections_topic").value
        self.min_marker_id = int(self.get_parameter("min_marker_id").value)
        self.max_marker_id = int(self.get_parameter("max_marker_id").value)
        self.publish_detections_enabled = bool(
            self.get_parameter("publish_detections").value
        )
        self.draw_rejected = bool(self.get_parameter("draw_rejected").value)
        self.log_interval_sec = float(self.get_parameter("log_interval_sec").value)

        dictionary_name = self.get_parameter("aruco_dictionary").value
        self.aruco_dict = self._load_aruco_dictionary(dictionary_name)
        self.aruco_detector = self._create_detector()

        self.bridge = CvBridge()
        self.last_log_time = 0.0

        self.debug_image_pub = self.create_publisher(Image, self.debug_image_topic, 10)
        self.detections_pub = self.create_publisher(String, self.detections_topic, 10)
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10
        )

        self.get_logger().info("Aruco detection demo node started.")
        self.get_logger().info(f"Listening image topic: {self.image_topic}")
        self.get_logger().info(f"Publishing debug image: {self.debug_image_topic}")
        self.get_logger().info(f"Publishing detections: {self.detections_topic}")

    def _load_aruco_dictionary(self, dictionary_name):
        dictionary_id = getattr(cv2.aruco, dictionary_name, None)
        if dictionary_id is None:
            self.get_logger().warn(
                f"Unknown ArUco dictionary '{dictionary_name}', using DICT_5X5_250."
            )
            dictionary_id = cv2.aruco.DICT_5X5_250
        return cv2.aruco.getPredefinedDictionary(dictionary_id)

    def _create_detector(self):
        if hasattr(cv2.aruco, "ArucoDetector"):
            self.aruco_params = cv2.aruco.DetectorParameters()
            return cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        return None

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"CvBridge error: {exc}")
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self._detect_markers(gray)

        detections = self._build_detections(corners, ids, cv_image.shape[1])
        self._draw_overlay(cv_image, detections, corners, ids, rejected)
        self._publish_debug_image(cv_image, msg)
        self._publish_detections(detections, msg)
        self._log_status(detections)

    def _detect_markers(self, gray_image):
        if self.aruco_detector is not None:
            return self.aruco_detector.detectMarkers(gray_image)
        return cv2.aruco.detectMarkers(
            gray_image,
            self.aruco_dict,
            parameters=self.aruco_params,
        )

    def _build_detections(self, corners, ids, image_width):
        detections = []
        if ids is None:
            return detections

        flattened_ids = ids.flatten()
        for marker_corners, marker_id in zip(corners, flattened_ids):
            marker_id = int(marker_id)
            if not self._is_marker_id_allowed(marker_id):
                continue

            points = marker_corners[0]
            center_x = float(np.mean(points[:, 0]))
            center_y = float(np.mean(points[:, 1]))
            area = float(cv2.contourArea(points.astype(np.float32)))
            normalized_error_x = (center_x - (image_width / 2.0)) / (image_width / 2.0)

            detections.append(
                {
                    "id": marker_id,
                    "center_x": center_x,
                    "center_y": center_y,
                    "area": area,
                    "normalized_error_x": normalized_error_x,
                    "corners": points.tolist(),
                }
            )

        detections.sort(key=lambda detection: detection["area"], reverse=True)
        return detections

    def _is_marker_id_allowed(self, marker_id):
        min_enabled = self.min_marker_id >= 0
        max_enabled = self.max_marker_id >= 0

        if min_enabled and marker_id < self.min_marker_id:
            return False
        if max_enabled and marker_id > self.max_marker_id:
            return False
        return True

    def _draw_overlay(self, cv_image, detections, corners, ids, rejected):
        if ids is not None and len(detections) > 0:
            valid_ids = np.array([[detection["id"]] for detection in detections])
            valid_corners = [
                np.array([detection["corners"]], dtype=np.float32)
                for detection in detections
            ]
            cv2.aruco.drawDetectedMarkers(cv_image, valid_corners, valid_ids)

        if self.draw_rejected and rejected is not None:
            cv2.aruco.drawDetectedMarkers(
                cv_image, rejected, borderColor=(80, 80, 80)
            )

        for detection in detections:
            center = (int(detection["center_x"]), int(detection["center_y"]))
            label_point = (center[0] + 10, center[1] - 10)

            cv2.circle(cv_image, center, 5, (0, 0, 255), -1)
            cv2.putText(
                cv_image,
                f"ID {detection['id']}",
                label_point,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

        status = "Aruco detected" if detections else "No Aruco marker"
        count_text = f"Count: {len(detections)}"
        self._draw_status_box(cv_image, status, count_text)

    def _draw_status_box(self, cv_image, status, count_text):
        box_height = 72
        cv2.rectangle(cv_image, (10, 10), (330, box_height), (0, 0, 0), -1)
        cv2.rectangle(cv_image, (10, 10), (330, box_height), (0, 255, 0), 2)
        cv2.putText(
            cv_image,
            status,
            (22, 38),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            cv_image,
            count_text,
            (22, 62),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

    def _publish_debug_image(self, cv_image, source_msg):
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"CvBridge debug image error: {exc}")
            return

        debug_msg.header = source_msg.header
        self.debug_image_pub.publish(debug_msg)

    def _publish_detections(self, detections, source_msg):
        if not self.publish_detections_enabled:
            return

        stamp = source_msg.header.stamp
        payload = {
            "timestamp": stamp.sec + stamp.nanosec * 1e-9,
            "frame_id": source_msg.header.frame_id,
            "count": len(detections),
            "detections": detections,
        }

        msg = String()
        msg.data = json.dumps(payload)
        self.detections_pub.publish(msg)

    def _log_status(self, detections):
        now = time.time()
        if now - self.last_log_time < self.log_interval_sec:
            return
        self.last_log_time = now

        if not detections:
            self.get_logger().info("No ArUco marker detected.")
            return

        ids = ", ".join(str(detection["id"]) for detection in detections)
        self.get_logger().info(f"Detected ArUco marker IDs: {ids}")


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectionDemo()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
