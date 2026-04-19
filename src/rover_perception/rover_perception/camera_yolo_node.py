"""ROS 2 node: webcam -> YOLOv8 TensorRT -> Detection2DArray + images."""
from __future__ import annotations

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class CameraYoloNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_yolo_node")

        self.declare_parameter("camera_index", 0)
        self.declare_parameter("model_path", "/workspaces/isaac_ros-dev/yolov8n.engine")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("confidence_threshold", 0.4)
        self.declare_parameter("yolo_imgsz", 640)
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("detections_topic", "/detections")

        cam_idx = int(self.get_parameter("camera_index").value)
        model_path = str(self.get_parameter("model_path").value)
        self.rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.img_w = int(self.get_parameter("image_width").value)
        self.img_h = int(self.get_parameter("image_height").value)
        self.conf_thresh = float(self.get_parameter("confidence_threshold").value)
        self.yolo_imgsz = int(self.get_parameter("yolo_imgsz").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        topic = str(self.get_parameter("detections_topic").value)

        self.get_logger().info(f"Loading YOLO model from {model_path}...")
        self.model = YOLO(model_path, task="detect")
        self.get_logger().info("Model loaded.")

        self.get_logger().info(f"Opening camera index {cam_idx}...")
        self.cap = cv2.VideoCapture(cam_idx)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.img_w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.img_h)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera at index {cam_idx}")
        self.get_logger().info("Camera opened.")

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Detection2DArray, topic, 10)
        self.image_pub = self.create_publisher(Image, "/camera/image_raw", 10)
        self.annotated_pub = self.create_publisher(Image, "/camera/image_annotated", 10)

        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)
        self.frame_count = 0

    def on_timer(self) -> None:
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("Failed to grab frame from camera.")
            return

        results = self.model.predict(
            frame,
            imgsz=self.yolo_imgsz,
            conf=self.conf_thresh,
            device=0,
            verbose=False,
        )
        if not results:
            return

        r = results[0]
        msg = Detection2DArray()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        boxes = r.boxes
        if boxes is not None and len(boxes) > 0:
            xyxy = boxes.xyxy.cpu().numpy()
            confs = boxes.conf.cpu().numpy()
            clses = boxes.cls.cpu().numpy().astype(int)
            names = r.names

            for i in range(len(xyxy)):
                x1 = float(xyxy[i][0])
                y1 = float(xyxy[i][1])
                x2 = float(xyxy[i][2])
                y2 = float(xyxy[i][3])
                conf = float(confs[i])
                cls_id = int(clses[i])
                label = names.get(cls_id, str(cls_id))

                det = Detection2D()
                det.bbox.center.position.x = (x1 + x2) / 2.0
                det.bbox.center.position.y = (y1 + y2) / 2.0
                det.bbox.size_x = x2 - x1
                det.bbox.size_y = y2 - y1

                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = label
                hypothesis.hypothesis.score = conf
                det.results.append(hypothesis)

                msg.detections.append(det)

        try:
            raw_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            raw_msg.header = msg.header
            self.image_pub.publish(raw_msg)

            annotated = frame.copy()
            for det in msg.detections:
                cx = det.bbox.center.position.x
                cy = det.bbox.center.position.y
                sx = det.bbox.size_x
                sy = det.bbox.size_y
                x1 = int(cx - sx / 2)
                y1 = int(cy - sy / 2)
                x2 = int(cx + sx / 2)
                y2 = int(cy + sy / 2)
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                if det.results:
                    label = f"{det.results[0].hypothesis.class_id} {det.results[0].hypothesis.score:.2f}"
                    cv2.putText(annotated, label, (x1, max(14, y1 - 6)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
            ann_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            ann_msg.header = msg.header
            self.annotated_pub.publish(ann_msg)
        except Exception as e:
            self.get_logger().warn(f"Image publish failed: {e}")

        self.pub.publish(msg)
        self.frame_count += 1
        if self.frame_count % 30 == 1:
            self.get_logger().info(
                f"Published frame {self.frame_count} with {len(msg.detections)} detections"
            )

    def destroy_node(self) -> bool:
        if self.cap is not None:
            self.cap.release()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CameraYoloNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
