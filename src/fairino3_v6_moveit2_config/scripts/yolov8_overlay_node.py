#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.exceptions import ParameterUninitializedException
from rclpy.parameter import Parameter
from rclpy.duration import Duration
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from vision_msgs.msg import BoundingBox2D
from vision_msgs.msg import BoundingBox3D
from vision_msgs.msg import Detection2D
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import Detection3D
from vision_msgs.msg import Detection3DArray
from vision_msgs.msg import ObjectHypothesisWithPose
from vision_msgs.msg import Point2D
from vision_msgs.msg import Pose2D
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from tf2_ros import Buffer
from tf2_ros import TransformListener
import math
import json
import copy

from cv_bridge import CvBridge


class Yolov8OverlayNode(Node):
    def __init__(self) -> None:
        super().__init__(
            "yolov8_overlay",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        image_in = self._get_or_declare("image_in", "/wrist_camera/image_raw")
        image_out = self._get_or_declare("image_out", "/yolo/dbg_image")
        detections_out = self._get_or_declare("detections_out", "/yolo/detections")
        detections3d_out = self._get_or_declare("detections3d_out", "/yolo/detections_3d")
        recognition_result_out = self._get_or_declare("recognition_result_out", "/yolo/recognition_result")
        grasp_state_topic = self._get_or_declare("grasp_state_topic", "/world_model/grasp_state")
        weights = self._get_or_declare("weights", "yolov8n.pt")
        self.detector_mode = str(self._get_or_declare("detector_mode", "yolo")).strip().lower()
        conf = float(self._get_or_declare("conf", 0.25))
        iou = float(self._get_or_declare("iou", 0.7))
        device = str(self._get_or_declare("device", "")).strip()
        self.forward_raw_when_error = bool(self._get_or_declare("forward_raw_when_error", True))
        pub_reliable = bool(self._get_or_declare("pub_reliable", True))
        self.draw_centers = bool(self._get_or_declare("draw_centers", True))
        self.draw_hud = bool(self._get_or_declare("draw_hud", True))
        self.classes = self._parse_classes(self._get_or_declare("classes", ""))
        # Task 12 is deliberately 2D-only.  Depth/TF fusion belongs to task 13.
        self.publish_3d = bool(self._get_or_declare("publish_3d", False))
        self.depth_image_topic = str(self._get_or_declare("depth_image", "/wrist_camera/depth/image_raw"))
        self.depth_info_topic = str(self._get_or_declare("depth_camera_info", "/wrist_camera/depth/camera_info"))
        self.target_frame = str(self._get_or_declare("target_frame", "world"))
        self.target_class = str(self._get_or_declare("target_class", "banana")).strip()
        target_detection_classes = str(
            self._get_or_declare("target_detection_classes", self.target_class)
        )
        self.target_detection_classes = {
            item.strip() for item in target_detection_classes.split(",") if item.strip()
        }
        self.target_object_id = str(self._get_or_declare("target_object_id", "banana-1")).strip()
        self.min_score = float(self._get_or_declare("min_score", 0.5))
        self.target_confirm_frames = max(1, int(self._get_or_declare("target_confirm_frames", 3)))
        self.freeze_target = bool(self._get_or_declare("freeze_target", True))
        self.depth_window = int(self._get_or_declare("depth_window", 5))
        self.depth_min = float(self._get_or_declare("depth_min", 0.05))
        self.depth_max = float(self._get_or_declare("depth_max", 5.0))
        self.depth_scale_16uc1 = float(self._get_or_declare("depth_scale_16uc1", 0.001))
        self.sync_tolerance_sec = float(self._get_or_declare("sync_tolerance_sec", 0.25))
        self.rgb_min_area_px = max(1.0, float(self._get_or_declare("rgb_min_area_px", 120.0)))
        if self.detector_mode not in {"yolo", "rgb"}:
            raise ValueError("detector_mode must be 'yolo' or 'rgb'")

        self.conf = conf
        self.iou = iou
        self.device = device if device else None
        self._detections_out = str(detections_out)
        self.bridge = CvBridge()
        self.model = None
        self._model_load_error = None
        self._weights = str(weights)
        self._last_status_text = ""
        self._reported_first_detection = False
        self._reported_first_3d = False
        self._reported_3d_wait_reasons: set[str] = set()
        self._reported_detected_state = False
        self._reported_localized_state = False
        self._target_label = None
        self._target_confirm_count = 0
        self._target_confirmed = False
        self._frozen_target_pose = None
        self._rgb_mask_pixels = 0
        self._rgb_max_saturation = 0
        self._latest_depth_msg: Image | None = None
        self._latest_depth_info: CameraInfo | None = None
        self._pending_depth_msgs: list[Image] = []
        self._pending_detection_arrays: list[Detection2DArray] = []
        self._tf_buffer = None
        if self.publish_3d:
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)

        pub_qos = qos_profile_sensor_data
        if pub_reliable:
            pub_qos = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
            )
        self.pub = self.create_publisher(Image, image_out, pub_qos)
        camera_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.sub = self.create_subscription(Image, image_in, self._on_image, camera_qos)
        if self.publish_3d:
            self.depth_sub = self.create_subscription(
                Image, self.depth_image_topic, self._on_depth, camera_qos
            )
            self.depth_info_sub = self.create_subscription(
                CameraInfo, self.depth_info_topic, self._on_depth_info, camera_qos
            )
        self.det_pub = self.create_publisher(
            Detection2DArray,
            str(detections_out),
            QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10, reliability=QoSReliabilityPolicy.RELIABLE),
        )
        self.recognition_result_pub = self.create_publisher(String, str(recognition_result_out), 10)
        self.grasp_state_pub = self.create_publisher(String, str(grasp_state_topic), 10)
        if self.publish_3d:
            self.target_point_camera_pub = self.create_publisher(PointStamped, "/yolo/target_point_camera", 10)
            self.target_point_pub = self.create_publisher(PointStamped, "/yolo/target_point", 10)
            target_pose_qos = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.target_pose_pub = self.create_publisher(
                PoseStamped, "/yolo/target_pose", target_pose_qos
            )
            self.det3d_pub = self.create_publisher(
                Detection3DArray,
                str(detections3d_out),
                QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10, reliability=QoSReliabilityPolicy.RELIABLE),
            )
        self.get_logger().info(
            f"2D detector ready: {image_in} -> {detections_out}, {image_out}; waiting for RGB frames"
        )

    def _get_or_declare(self, name: str, default):
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
            return default
        try:
            p = self.get_parameter(name)
            if p.type_ == Parameter.Type.NOT_SET:
                raise ParameterUninitializedException(name)
            return p.value
        except ParameterUninitializedException:
            self.set_parameters([Parameter(name=name, value=default)])
            return default

    @staticmethod
    def _parse_classes(value) -> list[int] | None:
        if value is None:
            return None
        if isinstance(value, int):
            return [value]
        if isinstance(value, (list, tuple)):
            result = []
            for v in value:
                if v is None:
                    continue
                if isinstance(v, int):
                    result.append(v)
                else:
                    s = str(v).strip()
                    if s:
                        result.append(int(s))
            return result if result else None

        text = str(value).strip()
        if text == "":
            return None
        if text.startswith("[") and text.endswith("]"):
            text = text[1:-1].strip()
            if text == "":
                return None
        result: list[int] = []
        for part in text.split(","):
            part = part.strip()
            if part == "":
                continue
            result.append(int(part))
        return result if result else None

    def _on_depth(self, msg: Image) -> None:
        self._latest_depth_msg = msg
        self._pending_depth_msgs.append(msg)
        del self._pending_depth_msgs[:-20]
        self._try_publish_synced_target()

    def _on_depth_info(self, msg: CameraInfo) -> None:
        self._latest_depth_info = msg
        self._try_publish_synced_target()

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @staticmethod
    def _quat_rotate(q: Quaternion, v: tuple[float, float, float]) -> tuple[float, float, float]:
        x, y, z = v
        qx = float(q.x)
        qy = float(q.y)
        qz = float(q.z)
        qw = float(q.w)

        uvx = qy * z - qz * y
        uvy = qz * x - qx * z
        uvz = qx * y - qy * x

        uuvx = qy * uvz - qz * uvy
        uuvy = qz * uvx - qx * uvz
        uuvz = qx * uvy - qy * uvx

        uvx *= 2.0 * qw
        uvy *= 2.0 * qw
        uvz *= 2.0 * qw
        uuvx *= 2.0
        uuvy *= 2.0
        uuvz *= 2.0

        return (x + uvx + uuvx, y + uvy + uuvy, z + uvz + uuvz)

    def _pick_best_detection(self, det_array: Detection2DArray) -> Detection2D | None:
        best_det = None
        best_score = -1.0

        for det in det_array.detections:
            if not det.results:
                continue
            hyp = det.results[0].hypothesis
            cls = str(hyp.class_id)
            score = float(hyp.score)
            if self.target_detection_classes and cls not in self.target_detection_classes:
                continue
            if score < self.min_score:
                continue
            if score > best_score:
                best_score = score
                best_det = det

        return best_det

    def _confirm_target_detection(self, det: Detection2D | None) -> bool:
        if det is None:
            self._target_label = None
            self._target_confirm_count = 0
            self._target_confirmed = False
            return False
        label = str(det.results[0].hypothesis.class_id)
        if label == self._target_label:
            self._target_confirm_count += 1
        else:
            self._target_label = label
            self._target_confirm_count = 1
        self._target_confirmed = self._target_confirm_count >= self.target_confirm_frames
        return self._target_confirmed

    def _publish_recognition_result(self, det_array: Detection2DArray) -> None:
        """Publish every raw YOLO label so target selection is auditable per frame."""
        detections = []
        for det in det_array.detections:
            if not det.results:
                continue
            hyp = det.results[0].hypothesis
            label = str(hyp.class_id)
            score = float(hyp.score)
            detections.append({
                "label": label,
                "score": round(score, 3),
                "selected_as_banana": label in self.target_detection_classes and score >= self.min_score,
                "center_px": [round(float(det.bbox.center.position.x), 1), round(float(det.bbox.center.position.y), 1)],
            })
        message = String()
        message.data = json.dumps({
            "stamp": {"sec": det_array.header.stamp.sec, "nanosec": det_array.header.stamp.nanosec},
            "allowed_target_labels": sorted(self.target_detection_classes),
            "min_score": self.min_score,
            "target_confirm_frames": self.target_confirm_frames,
            "target_confirmed": self._target_confirmed,
            "rgb_mask_pixels": self._rgb_mask_pixels if self.detector_mode == "rgb" else None,
            "rgb_max_saturation": self._rgb_max_saturation if self.detector_mode == "rgb" else None,
            "detections": detections,
        }, separators=(",", ":"))
        self.recognition_result_pub.publish(message)

    def _append_rgb_banana_detection(self, frame, header, det_array: Detection2DArray):
        """Detect the scene's only yellow object with HSV segmentation."""
        import cv2

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (18, 80, 80), (40, 255, 255))
        self._rgb_mask_pixels = int(cv2.countNonZero(mask))
        self._rgb_max_saturation = int(hsv[:, :, 1].max())
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return frame, 0
        contour = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(contour))
        min_area = self.rgb_min_area_px * (frame.shape[0] * frame.shape[1]) / (320.0 * 240.0)
        if area < min_area:
            return frame, 0
        x, y, width, height = cv2.boundingRect(contour)
        det = Detection2D()
        det.header = header
        det.id = "rgb-yellow-0"
        det.bbox = BoundingBox2D()
        det.bbox.center = Pose2D(position=Point2D(x=x + width / 2.0, y=y + height / 2.0), theta=0.0)
        det.bbox.size_x = float(width)
        det.bbox.size_y = float(height)
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = "banana_rgb"
        hyp.hypothesis.score = min(0.99, area / min_area)
        det.results.append(hyp)
        det_array.detections.append(det)
        cv2.rectangle(frame, (x, y), (x + width, y + height), (0, 255, 0), 2)
        cv2.putText(frame, f"banana_rgb {hyp.hypothesis.score:.2f}", (x, max(18, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        return frame, 1

    def _depth_at_pixel_m(self, u: int, v: int, depth_msg: Image) -> float | None:
        try:
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception:
            return None

        if depth is None:
            return None
        h, w = depth.shape[:2]
        if u < 0 or v < 0 or u >= w or v >= h:
            return None

        def convert(val) -> float | None:
            if val is None:
                return None
            if hasattr(val, "item"):
                val = val.item()
            if isinstance(val, (int, float)):
                if isinstance(val, int):
                    if val == 0:
                        return None
                    z = float(val) * self.depth_scale_16uc1
                else:
                    if math.isnan(val) or val <= 0.0:
                        return None
                    z = float(val)
                if z < self.depth_min or z > self.depth_max:
                    return None
                return z
            return None

        z0 = convert(depth[v, u])
        if z0 is not None:
            return z0

        win = max(1, int(self.depth_window))
        if win % 2 == 0:
            win += 1
        r = win // 2
        samples: list[float] = []
        for dv in range(-r, r + 1):
            y = v + dv
            if y < 0 or y >= h:
                continue
            for du in range(-r, r + 1):
                x = u + du
                if x < 0 or x >= w:
                    continue
                z = convert(depth[y, x])
                if z is not None:
                    samples.append(z)
        if not samples:
            return None
        samples.sort()
        return samples[len(samples) // 2]

    @staticmethod
    def _bbox_size_m(det: Detection2D, fx: float, fy: float, z: float) -> tuple[float, float, float]:
        width = max(0.001, float(det.bbox.size_x) / fx * z)
        height = max(0.001, float(det.bbox.size_y) / fy * z)
        # ponytail: a single RGB-D view cannot observe thickness; use the smaller image-plane extent.
        return width, height, max(0.01, min(width, height))

    def _publish_3d_detection(
        self, header, pose: PoseStamped, det: Detection2D, dimensions: tuple[float, float, float]
    ) -> None:
        result = Detection3DArray()
        result.header = header
        result.header.frame_id = pose.header.frame_id
        detection = Detection3D()
        detection.header = result.header
        detection.id = self.target_object_id
        detection.bbox = BoundingBox3D()
        detection.bbox.center = pose.pose
        detection.bbox.size.x, detection.bbox.size.y, detection.bbox.size.z = dimensions
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = self.target_class
        hypothesis.hypothesis.score = float(det.results[0].hypothesis.score)
        detection.results.append(hypothesis)
        result.detections.append(detection)
        self.det3d_pub.publish(result)
        self._publish_grasp_state("localized", "depth_tf_fused", result.header)
        if not self._reported_first_3d:
            self.get_logger().info(
                f"first 3D localization published: {detection.id} in {result.header.frame_id}; "
                "subsequent target poses continue without repeated log messages"
            )
            self._reported_first_3d = True

    def _publish_grasp_state(self, state: str, reason: str, header) -> None:
        if state == "detected":
            if self._reported_detected_state:
                return
            self._reported_detected_state = True
        elif state == "localized":
            if self._reported_localized_state:
                return
            self._reported_localized_state = True
        message = String()
        message.data = json.dumps(
            {
                "object_id": self.target_object_id,
                "state": state,
                "reason": reason,
                "stamp": {"sec": header.stamp.sec, "nanosec": header.stamp.nanosec},
            },
            separators=(",", ":"),
        )
        self.grasp_state_pub.publish(message)

    def _try_publish_synced_target(self) -> None:
        if not self.publish_3d:
            return
        if self._latest_depth_info is None:
            self._report_3d_wait_once("waiting for depth image and CameraInfo")
            return
        best = None
        for det_index, det_array in enumerate(self._pending_detection_arrays):
            t_det = self._stamp_to_sec(det_array.header.stamp)
            for depth_index, depth_msg in enumerate(self._pending_depth_msgs):
                diff = abs(t_det - self._stamp_to_sec(depth_msg.header.stamp))
                if diff <= self.sync_tolerance_sec and (best is None or diff < best[0]):
                    best = (diff, det_index, depth_index)
        if best is None:
            return
        _, det_index, depth_index = best
        det_array = self._pending_detection_arrays.pop(det_index)
        depth_msg = self._pending_depth_msgs.pop(depth_index)
        self._publish_3d_target(det_array, depth_msg)

    def _publish_3d_target(self, det_array: Detection2DArray, depth_msg: Image) -> None:
        if not self.publish_3d:
            return

        if self._frozen_target_pose is not None:
            self.target_pose_pub.publish(self._frozen_target_pose)
            return

        depth_info = self._latest_depth_info
        if depth_info is None:
            self._report_3d_wait_once("waiting for depth image and CameraInfo")
            return

        det = self._pick_best_detection(det_array)
        if not self._confirm_target_detection(det):
            if det is not None:
                self._report_3d_wait_once(
                    f"waiting for stable {self._target_label} detection "
                    f"({self._target_confirm_count}/{self.target_confirm_frames} frames)"
                )
                return
            candidates = ", ".join(
                f"{d.results[0].hypothesis.class_id}@{d.results[0].hypothesis.score:.2f}"
                for d in det_array.detections if d.results
            ) or "none"
            self._report_3d_wait_once(
                f"no target class {sorted(self.target_detection_classes)!r} above "
                f"min_score={self.min_score:.2f}; candidates: {candidates}"
            )
            return

        u = int(round(float(det.bbox.center.position.x)))
        v = int(round(float(det.bbox.center.position.y)))
        z = self._depth_at_pixel_m(u, v, depth_msg)
        if z is None:
            self._report_3d_wait_once(f"no valid depth at target pixel ({u}, {v})")
            return

        k = list(depth_info.k)
        if len(k) != 9:
            self._report_3d_wait_once("CameraInfo K matrix is invalid")
            return
        fx = float(k[0])
        fy = float(k[4])
        cx0 = float(k[2])
        cy0 = float(k[5])
        if fx == 0.0 or fy == 0.0:
            self._report_3d_wait_once("CameraInfo focal length is zero")
            return

        x = (float(u) - cx0) / fx * z
        y = (float(v) - cy0) / fy * z
        dimensions = self._bbox_size_m(det, fx, fy, z)

        camera_frame = str(depth_info.header.frame_id or depth_msg.header.frame_id or det_array.header.frame_id)
        p_cam = PointStamped()
        p_cam.header = det_array.header
        p_cam.header.frame_id = camera_frame
        p_cam.point.x = float(x)
        p_cam.point.y = float(y)
        p_cam.point.z = float(z)
        self.target_point_camera_pub.publish(p_cam)

        tf = None
        try:
            tf = self._tf_buffer.lookup_transform(
                self.target_frame,
                camera_frame,
                rclpy.time.Time.from_msg(det_array.header.stamp),
                timeout=Duration(seconds=0.2),
            )
        except Exception:
            try:
                tf = self._tf_buffer.lookup_transform(
                    self.target_frame,
                    camera_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.2),
                )
            except Exception:
                self._report_3d_wait_once(
                    f"TF unavailable: {self.target_frame} <- {camera_frame}"
                )
                return

        rot = tf.transform.rotation
        trans = tf.transform.translation
        x2, y2, z2 = self._quat_rotate(rot, (p_cam.point.x, p_cam.point.y, p_cam.point.z))
        x2 += float(trans.x)
        y2 += float(trans.y)
        z2 += float(trans.z)

        p_out = PointStamped()
        p_out.header = det_array.header
        p_out.header.frame_id = self.target_frame
        p_out.point.x = float(x2)
        p_out.point.y = float(y2)
        p_out.point.z = float(z2)
        self.target_point_pub.publish(p_out)

        pose = PoseStamped()
        pose.header = p_out.header
        pose.pose.position.x = p_out.point.x
        pose.pose.position.y = p_out.point.y
        pose.pose.position.z = p_out.point.z
        pose.pose.orientation.w = 1.0
        self._publish_target_pose(pose)
        self._publish_3d_detection(det_array.header, pose, det, dimensions)

    def _publish_target_pose(self, pose: PoseStamped) -> None:
        if self.freeze_target and self._frozen_target_pose is None:
            self._frozen_target_pose = copy.deepcopy(pose)
            self.get_logger().info(
                "RGB target frozen in "
                f"{pose.header.frame_id}: ({pose.pose.position.x:.3f}, "
                f"{pose.pose.position.y:.3f}, {pose.pose.position.z:.3f})"
            )
        self.target_pose_pub.publish(self._frozen_target_pose or pose)

    def _report_3d_wait_once(self, reason: str) -> None:
        if self._reported_first_3d or reason in self._reported_3d_wait_reasons:
            return
        self._reported_3d_wait_reasons.add(reason)
        self.get_logger().info(f"3D localization waiting: {reason}")

    def _on_image(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
                import cv2

                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            except Exception:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        annotated = None
        det_count = None
        det_array = Detection2DArray()
        det_array.header = msg.header
        if self.detector_mode == "yolo" and self.model is None and self._model_load_error is None:
            try:
                from ultralytics import YOLO

                self.model = YOLO(self._weights)
                self._names = getattr(self.model, "names", None)
                self.get_logger().info(f"YOLOv8 model loaded: {self._weights}")
            except Exception as exc:
                self._model_load_error = str(exc)
                self.get_logger().error(f"YOLOv8 load failed: {self._model_load_error}")

        if self.detector_mode == "yolo" and self.model is not None:
            try:
                results = self.model.predict(
                    source=frame,
                    conf=self.conf,
                    iou=self.iou,
                    device=self.device,
                    classes=self.classes,
                    verbose=False,
                )
                r0 = results[0]
                annotated = r0.plot()
                if getattr(r0, "boxes", None) is not None:
                    import cv2

                    boxes = r0.boxes
                    xyxy = boxes.xyxy.cpu().numpy() if boxes.xyxy is not None else None
                    cls = boxes.cls.cpu().numpy() if boxes.cls is not None else None
                    conf = boxes.conf.cpu().numpy() if boxes.conf is not None else None
                    if xyxy is not None:
                        det_count = int(xyxy.shape[0])
                        detections: list[Detection2D] = []
                        for idx, (x1, y1, x2, y2) in enumerate(xyxy):
                            try:
                                cx = float((x1 + x2) / 2.0)
                                cy = float((y1 + y2) / 2.0)

                                det = Detection2D()
                                det.header = msg.header
                                det.id = str(idx)
                                det.bbox = BoundingBox2D()
                                det.bbox.center = Pose2D(position=Point2D(x=cx, y=cy), theta=0.0)
                                det.bbox.size_x = float(max(0.0, x2 - x1))
                                det.bbox.size_y = float(max(0.0, y2 - y1))

                                hyp = ObjectHypothesisWithPose()
                                cls_id = int(cls[idx]) if cls is not None else -1
                                names = getattr(self, "_names", None)
                                if isinstance(names, dict) and cls_id in names:
                                    hyp.hypothesis.class_id = str(names[cls_id])
                                else:
                                    hyp.hypothesis.class_id = str(cls_id)
                                hyp.hypothesis.score = float(conf[idx]) if conf is not None else 0.0
                                det.results.append(hyp)
                                detections.append(det)

                                label = str(hyp.hypothesis.class_id)
                                is_target = label in self.target_detection_classes and float(hyp.hypothesis.score) >= self.min_score
                                if is_target:
                                    cv2.rectangle(
                                        annotated,
                                        (int(x1), int(y1)),
                                        (int(x2), int(y2)),
                                        (0, 255, 0),
                                        2,
                                    )
                                if self.draw_centers:
                                    center_color = (0, 255, 0) if is_target else (0, 0, 255)
                                    cv2.circle(annotated, (int(cx), int(cy)), 4, center_color, -1)
                            except Exception:
                                import traceback

                                self.get_logger().error(
                                    "Failed to build Detection2D for one box:\n" + traceback.format_exc()
                                )
                        det_array.detections = detections
                    if det_count is None:
                        det_count = int(len(boxes)) if boxes is not None else 0
            except Exception as exc:
                import traceback

                self.get_logger().error("YOLOv8 inference failed:\n" + traceback.format_exc())

        if self.detector_mode == "rgb":
            annotated, det_count = self._append_rgb_banana_detection(frame.copy(), msg.header, det_array)

        if annotated is None:
            if not self.forward_raw_when_error:
                return
            annotated = frame
            det_count = det_count if det_count is not None else 0

        if self.draw_hud:
            import cv2

            det_text = "?"
            if det_count is not None:
                det_text = str(det_count)
            model_state = "RGB" if self.detector_mode == "rgb" else ("OK" if self.model is not None else ("ERR" if self._model_load_error else "LOADING"))
            hud1 = f"dets={det_text} conf={self.conf:.2f} model={model_state}"
            cv2.putText(annotated, hud1, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 0), 3)
            cv2.putText(annotated, hud1, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 1)

            status2 = ""
            if self._model_load_error:
                status2 = f"load_error={self._model_load_error}"
            if status2:
                status2 = status2.replace("\n", " ")[:120]
                cv2.putText(annotated, status2, (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
                cv2.putText(annotated, status2, (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

        out = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        out.header = msg.header
        self.pub.publish(out)
        self.det_pub.publish(det_array)
        self._pending_detection_arrays.append(det_array)
        del self._pending_detection_arrays[:-20]
        self._try_publish_synced_target()
        self._publish_recognition_result(det_array)
        if self._target_confirmed:
            self._publish_grasp_state("detected", "2d_detector", det_array.header)
        if not self._reported_first_detection:
            self.get_logger().info(
                f"2D detection published: {len(det_array.detections)} boxes on {self._detections_out}"
            )
            self._reported_first_detection = True


def main() -> None:
    rclpy.init()
    node = Yolov8OverlayNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
