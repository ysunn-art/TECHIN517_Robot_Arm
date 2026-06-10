import abc
import numpy as np
import cv2
from dataclasses import dataclass, field
from typing import Any, Optional
from functools import partial

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from scipy.spatial.transform import Rotation as R

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
import tf2_ros


# ---------------------------------------------------------------------------
# Image conversion helpers (no cv_bridge needed)
# ---------------------------------------------------------------------------

_ENCODING_INFO = {
    'mono8': (np.uint8, 1),
    'bgr8': (np.uint8, 3),
    'rgb8': (np.uint8, 3),
    'bgra8': (np.uint8, 4),
    'rgba8': (np.uint8, 4),
    '16UC1': (np.uint16, 1),
    '32FC1': (np.float32, 1),
}


def imgmsg_to_cv2(msg: Image) -> np.ndarray:
    """Convert a ROS Image message to a BGR OpenCV image."""
    dtype, channels = _ENCODING_INFO.get(msg.encoding, (np.uint8, 3))
    if channels == 1:
        img = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width)
    else:
        img = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, channels)

    if msg.encoding == 'rgb8':
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if msg.encoding == 'rgba8':
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
    if msg.encoding == 'bgra8':
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    return img


def cv2_to_imgmsg(cv_image: np.ndarray, encoding: str = 'bgr8') -> Image:
    """Convert an OpenCV image to a ROS Image message."""
    msg = Image()
    msg.height, msg.width = cv_image.shape[:2]
    msg.encoding = encoding
    msg.step = int(cv_image.strides[0])
    msg.data = cv_image.tobytes()
    return msg


# ---------------------------------------------------------------------------
# ArUco helper functions
# ---------------------------------------------------------------------------

def compute_obj_points(marker_size: float) -> np.ndarray:
    """Return the 4x3 float32 array of 3D corner points for a square marker."""
    half = marker_size / 2.0
    return np.array([
        [-half,  half, 0.0],
        [ half,  half, 0.0],
        [ half, -half, 0.0],
        [-half, -half, 0.0],
    ], dtype=np.float32)


def estimate_marker_pose(
    obj_points: np.ndarray,
    image_corners: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    pnp_flags: int = cv2.SOLVEPNP_IPPE_SQUARE,
) -> tuple:
    """
    Run solvePnP and compute reprojection error.

    Returns (success, rvec, tvec, reprojection_error).
    On failure rvec/tvec are None and error is inf.
    """
    success, rvec, tvec = cv2.solvePnP(
        obj_points,
        image_corners,
        camera_matrix,
        dist_coeffs,
        flags=pnp_flags,
    )
    if not success:
        return False, None, None, float('inf')

    projected, _ = cv2.projectPoints(
        obj_points, rvec, tvec, camera_matrix, dist_coeffs
    )
    error = np.mean(np.linalg.norm(
        projected.reshape(4, 2) - image_corners, axis=1
    ))
    return True, rvec, tvec, error


def build_transform_matrix(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    """Convert rvec, tvec from solvePnP into a 4x4 homogeneous transform."""
    rot_mat, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = rot_mat
    T[:3, 3] = tvec.flatten()
    return T


def make_image_qos() -> QoSProfile:
    """Return a BEST_EFFORT QoS profile suitable for image subscriptions."""
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
    )


# ---------------------------------------------------------------------------
# Per-camera state
# ---------------------------------------------------------------------------

@dataclass
class CameraState:
    name: str
    image_topic: str
    camera_info_topic: str
    camera_matrix: Optional[np.ndarray] = None
    dist_coeffs: Optional[np.ndarray] = None
    frame_id: Optional[str] = None
    info_received: bool = False
    debug_pub: Any = None


# ---------------------------------------------------------------------------
# Base class for ArUco tracker nodes
# ---------------------------------------------------------------------------

class ArucoTrackerBase(Node, abc.ABC):
    """
    Base class for ArUco tracker nodes.

    Handles shared parameter declaration, ArUco detector setup, multi-camera
    subscription wiring, camera info storage, TF broadcasting, and debug
    image publishing.

    Subclasses MUST implement:
        _process_detections(cam, cv_image, corners, ids, header)
    """

    def __init__(self, node_name: str):
        super().__init__(node_name)

        # Declare shared parameters
        self.declare_parameter('image_topics', ['/camera/image_raw'])
        self.declare_parameter('camera_info_topics', ['/camera/camera_info'])
        self.declare_parameter('camera_names', ['camera'])
        self.declare_parameter('marker_size', 0.05)
        self.declare_parameter('aruco_dictionary', 0)  # DICT_4X4_50
        self.declare_parameter('publish_debug_images', True)

        # Read shared parameters
        image_topics = self.get_parameter('image_topics').get_parameter_value().string_array_value
        camera_info_topics = self.get_parameter('camera_info_topics').get_parameter_value().string_array_value
        camera_names = self.get_parameter('camera_names').get_parameter_value().string_array_value
        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value
        aruco_dict_id = self.get_parameter('aruco_dictionary').get_parameter_value().integer_value
        self.publish_debug = self.get_parameter('publish_debug_images').get_parameter_value().bool_value

        # Validate parameter lengths
        if not (len(image_topics) == len(camera_info_topics) == len(camera_names)):
            self.get_logger().fatal(
                'image_topics, camera_info_topics, and camera_names must have the same length. '
                f'Got {len(image_topics)}, {len(camera_info_topics)}, {len(camera_names)}.'
            )
            raise ValueError('Mismatched parameter array lengths')

        # OpenCV ArUco setup
        dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict_id)
        detector_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(dictionary, detector_params)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 3D object points for solvePnP
        self.obj_points = compute_obj_points(self.marker_size)

        # Set up per-camera state and subscriptions
        image_qos = make_image_qos()
        self.cameras: list[CameraState] = []

        for name, img_topic, info_topic in zip(camera_names, image_topics, camera_info_topics):
            cam = CameraState(
                name=name,
                image_topic=img_topic,
                camera_info_topic=info_topic,
            )

            self.create_subscription(
                CameraInfo,
                info_topic,
                partial(self._camera_info_callback, cam=cam),
                10,
            )

            self.create_subscription(
                Image,
                img_topic,
                partial(self._image_callback, cam=cam),
                image_qos,
            )

            if self.publish_debug:
                cam.debug_pub = self.create_publisher(Image, f'~/debug/{name}', 1)

            self.cameras.append(cam)
            self.get_logger().info(f'Camera "{name}": image={img_topic}, info={info_topic}')

    def _camera_info_callback(self, msg: CameraInfo, cam: CameraState):
        """Store camera intrinsics from CameraInfo message."""
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        if K[0, 0] == 0.0:
            if not cam.info_received:
                self.get_logger().warn(
                    f'Camera "{cam.name}": received CameraInfo with zero focal length. '
                    'Camera may not be calibrated.'
                )
            return

        cam.camera_matrix = K
        cam.dist_coeffs = np.array(msg.d, dtype=np.float64)
        cam.frame_id = msg.header.frame_id

        if not cam.info_received:
            cam.info_received = True
            self.get_logger().info(
                f'Camera "{cam.name}": received calibration. '
                f'Frame: "{cam.frame_id}", fx={K[0,0]:.1f}, fy={K[1,1]:.1f}'
            )

    def _image_callback(self, msg: Image, cam: CameraState):
        """Detect ArUco markers and delegate to subclass for processing."""
        if not cam.info_received:
            return

        try:
            cv_image = imgmsg_to_cv2(msg)
        except Exception as e:
            self.get_logger().error(f'Camera "{cam.name}": image conversion error: {e}')
            return

        corners, ids, _ = self.aruco_detector.detectMarkers(cv_image)

        if ids is None or len(ids) == 0:
            if self.publish_debug and cam.debug_pub is not None:
                self.publish_debug_image(cam, cv_image, msg.header)
            return

        self._process_detections(cam, cv_image, corners, ids, msg.header)

    @abc.abstractmethod
    def _process_detections(self, cam: CameraState, cv_image: np.ndarray,
                            corners: tuple, ids: np.ndarray, header) -> None:
        """Subclasses implement marker-specific processing here."""

    def broadcast_tf(self, T: np.ndarray, parent_frame: str,
                     child_frame: str, stamp) -> None:
        """Broadcast a 4x4 homogeneous transform as a TF frame."""
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

        t.transform.translation.x = float(T[0, 3])
        t.transform.translation.y = float(T[1, 3])
        t.transform.translation.z = float(T[2, 3])

        quat = R.from_matrix(T[:3, :3]).as_quat()  # [x, y, z, w]
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])

        self.tf_broadcaster.sendTransform(t)

    def publish_debug_image(self, cam: CameraState, cv_image: np.ndarray, header) -> None:
        """Publish a debug image on the camera's debug publisher."""
        try:
            debug_msg = cv2_to_imgmsg(cv_image, encoding='bgr8')
            debug_msg.header = header
            cam.debug_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'Camera "{cam.name}": debug publish error: {e}')
