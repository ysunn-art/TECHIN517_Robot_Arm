import numpy as np
import cv2

import rclpy

from aruco_tracker_ros2.aruco_utils import (
    ArucoTrackerBase,
    CameraState,
    estimate_marker_pose,
    build_transform_matrix,
)


class ArucoCodeTracker(ArucoTrackerBase):
    """Tracks individual ArUco markers and publishes their poses as TF frames."""

    def __init__(self):
        super().__init__('aruco_code_tracker')

        # Single-marker-specific parameters
        self.declare_parameter('marker_id', -1)
        self.declare_parameter('marker_frame_id', 'aruco_code')

        self.target_marker_id = self.get_parameter('marker_id').get_parameter_value().integer_value
        self.marker_frame_id = self.get_parameter('marker_frame_id').get_parameter_value().string_value

        mode = "all markers" if self.target_marker_id == -1 else f"marker ID {self.target_marker_id}"
        self.get_logger().info(
            f'ArUco code tracker started. Tracking {mode} across {len(self.cameras)} camera(s). '
            f'Marker size={self.marker_size}m, frame prefix="{self.marker_frame_id}"'
        )

    def _process_detections(self, cam, cv_image, corners, ids, header):
        """Estimate pose for each tracked marker and broadcast TF."""
        for i, marker_id in enumerate(ids.flatten()):
            if self.target_marker_id != -1 and marker_id != self.target_marker_id:
                continue

            marker_corners = corners[i].reshape(4, 2).astype(np.float64)

            success, rvec, tvec, error = estimate_marker_pose(
                self.obj_points, marker_corners,
                cam.camera_matrix, cam.dist_coeffs,
            )
            if not success:
                continue

            T_cam_marker = build_transform_matrix(rvec, tvec)

            if self.target_marker_id == -1:
                child_frame = f"{self.marker_frame_id}_{marker_id}"
            else:
                child_frame = self.marker_frame_id

            self.broadcast_tf(T_cam_marker, cam.frame_id, child_frame, header.stamp)

        if self.publish_debug and cam.debug_pub is not None:
            self._draw_and_publish_debug(cam, cv_image, corners, ids, header)

    def _draw_and_publish_debug(self, cam, cv_image, corners, ids, header):
        """Draw detected markers and axes for tracked ones."""
        debug_img = cv_image.copy()

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(debug_img, corners, ids)

            for i, marker_id in enumerate(ids.flatten()):
                if self.target_marker_id != -1 and marker_id != self.target_marker_id:
                    continue

                marker_corners = corners[i].reshape(4, 2).astype(np.float64)
                success, rvec, tvec, _ = estimate_marker_pose(
                    self.obj_points, marker_corners,
                    cam.camera_matrix, cam.dist_coeffs,
                )
                if success:
                    cv2.drawFrameAxes(
                        debug_img, cam.camera_matrix, cam.dist_coeffs,
                        rvec, tvec, self.marker_size * 0.75
                    )

        self.publish_debug_image(cam, debug_img, header)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoCodeTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
