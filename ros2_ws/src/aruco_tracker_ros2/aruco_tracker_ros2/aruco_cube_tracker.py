import numpy as np
import cv2

import rclpy
from scipy.spatial.transform import Rotation as R

from aruco_tracker_ros2.aruco_utils import (
    ArucoTrackerBase,
    CameraState,
    estimate_marker_pose,
    build_transform_matrix,
)


class ArucoCubeTracker(ArucoTrackerBase):

    # Face ID -> rotation from marker frame to canonical cube frame
    # Cube frame is aligned with front face (ID 2):
    #   +Z = front outward, +X = right, +Y = up
    FACE_ROTATIONS = {
        2: R.identity(),                          # Front
        3: R.from_euler('y', 180, degrees=True),  # Back
        1: R.from_euler('y', -90, degrees=True),  # Right
        4: R.from_euler('y', 90, degrees=True),   # Left
        5: R.from_euler('x', 90, degrees=True),   # Top
        0: R.from_euler('x', -90, degrees=True),  # Bottom
    }

    def __init__(self):
        super().__init__('aruco_cube_tracker')

        # Cube-specific parameters
        self.declare_parameter('cube_size', 0.03)
        self.declare_parameter('cube_frame_id', 'aruco_cube')

        self.cube_size = self.get_parameter('cube_size').get_parameter_value().double_value
        self.cube_frame_id = self.get_parameter('cube_frame_id').get_parameter_value().string_value

        self._build_marker_to_cube_transforms()

        self.get_logger().info(
            f'ArUco cube tracker started. Tracking {len(self.cameras)} camera(s). '
            f'Marker size={self.marker_size}m, cube size={self.cube_size}m, '
            f'frame="{self.cube_frame_id}"'
        )

    def _build_marker_to_cube_transforms(self):
        """Precompute 4x4 homogeneous transforms from each marker frame to cube center frame."""
        half_cube = self.cube_size / 2.0
        t_marker_cube = np.array([0.0, 0.0, -half_cube])

        self.marker_to_cube = {}
        for marker_id, rot in self.FACE_ROTATIONS.items():
            T = np.eye(4)
            T[:3, :3] = rot.as_matrix()
            T[:3, 3] = t_marker_cube
            self.marker_to_cube[marker_id] = T

    def _process_detections(self, cam, cv_image, corners, ids, header):
        """Select best marker by reprojection error, compute cube center, broadcast TF."""
        best_T_cam_cube = None
        best_error = float('inf')

        for i, marker_id in enumerate(ids.flatten()):
            if marker_id not in self.marker_to_cube:
                continue

            marker_corners = corners[i].reshape(4, 2).astype(np.float64)

            success, rvec, tvec, error = estimate_marker_pose(
                self.obj_points, marker_corners,
                cam.camera_matrix, cam.dist_coeffs,
            )
            if not success:
                continue

            T_cam_marker = build_transform_matrix(rvec, tvec)
            T_cam_cube = T_cam_marker @ self.marker_to_cube[marker_id]

            if error < best_error:
                best_error = error
                best_T_cam_cube = T_cam_cube

        if best_T_cam_cube is not None:
            self.broadcast_tf(best_T_cam_cube, cam.frame_id, self.cube_frame_id, header.stamp)

        if self.publish_debug and cam.debug_pub is not None:
            self._draw_and_publish_debug(cam, cv_image, corners, ids, best_T_cam_cube, header)

    def _draw_and_publish_debug(self, cam, cv_image, corners, ids, T_cam_cube, header):
        """Draw detected markers and cube center on the image and publish."""
        debug_img = cv_image.copy()

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(debug_img, corners, ids)

            for i, marker_id in enumerate(ids.flatten()):
                if marker_id not in self.marker_to_cube:
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

        if T_cam_cube is not None:
            cube_center_3d = T_cam_cube[:3, 3].reshape(1, 1, 3)
            rvec_identity = np.zeros((3, 1))
            tvec_zero = np.zeros((3, 1))
            projected, _ = cv2.projectPoints(
                cube_center_3d, rvec_identity, tvec_zero,
                cam.camera_matrix, cam.dist_coeffs
            )
            cx, cy = int(projected[0, 0, 0]), int(projected[0, 0, 1])
            cv2.drawMarker(
                debug_img, (cx, cy), (0, 0, 255),
                cv2.MARKER_CROSS, 20, 2
            )
            cv2.putText(
                debug_img, 'CUBE', (cx + 10, cy - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2
            )

        self.publish_debug_image(cam, debug_img, header)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoCubeTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
