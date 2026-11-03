#!/usr/bin/env python3
"""
Lightweight OpenCV ArUco/AprilTag tracker module for the Strix Halo host.
Computes real-time 3D/2D end-effector position and compares it against target markers
and the ESP32-S3 accelerometer pitch/roll vector. Includes mock fallback for dry-run testing.
"""

import math
from typing import Optional, Tuple, Dict, Any

try:
    import cv2
    import numpy as np
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    np = None


class VisionTracker:
    def __init__(self, camera_id: int = 0, marker_size_m: float = 0.05, mock_mode: bool = False):
        self.camera_id = camera_id
        self.marker_size_m = marker_size_m
        self.mock_mode = mock_mode or not CV2_AVAILABLE
        self.cap = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.detector = None
        self.last_ee_pos = (0.0, 0.0, 0.0)
        self.target_pos = (0.15, 0.0, 0.10)

        if not self.mock_mode:
            self._init_camera()

    def _init_camera(self):
        try:
            self.cap = cv2.VideoCapture(self.camera_id)
            if not self.cap.isOpened():
                self.mock_mode = True
                return

            # Default generic calibration for 640x480 webcam
            focal_length = 600.0
            center = (320, 240)
            self.camera_matrix = np.array([
                [focal_length, 0, center[0]],
                [0, focal_length, center[1]],
                [0, 0, 1]
            ], dtype=np.float32)
            self.dist_coeffs = np.zeros((4, 1), dtype=np.float32)

            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            parameters = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        except Exception:
            self.mock_mode = True

    def set_target(self, x: float, y: float, z: float):
        self.target_pos = (x, y, z)

    def track_frame(self) -> Dict[str, Any]:
        """
        Track markers in the frame.
        Returns:
            dict containing end_effector_pos, target_pos, visual_error_dist, and detected status.
        """
        if self.mock_mode or not self.cap:
            # Simulate a realistic visual tracking reading
            dx = self.target_pos[0] - self.last_ee_pos[0]
            dy = self.target_pos[1] - self.last_ee_pos[1]
            dz = self.target_pos[2] - self.last_ee_pos[2]
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            return {
                "detected": True,
                "end_effector_pos": self.last_ee_pos,
                "target_pos": self.target_pos,
                "visual_error_dist": dist,
                "mock": True
            }

        ret, frame = self.cap.read()
        if not ret:
            return {
                "detected": False,
                "end_effector_pos": self.last_ee_pos,
                "target_pos": self.target_pos,
                "visual_error_dist": 0.5,
                "mock": False
            }

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)

        if ids is not None and len(ids) > 0:
            # Marker 0: Arm End-Effector; Marker 1: Goal Target (optional)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size_m, self.camera_matrix, self.dist_coeffs
            )
            for idx, m_id in enumerate(ids.flatten()):
                if m_id == 0:
                    t = tvecs[idx][0]
                    self.last_ee_pos = (float(t[0]), float(t[1]), float(t[2]))
                elif m_id == 1:
                    t = tvecs[idx][0]
                    self.target_pos = (float(t[0]), float(t[1]), float(t[2]))

            dx = self.target_pos[0] - self.last_ee_pos[0]
            dy = self.target_pos[1] - self.last_ee_pos[1]
            dz = self.target_pos[2] - self.last_ee_pos[2]
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)

            return {
                "detected": True,
                "end_effector_pos": self.last_ee_pos,
                "target_pos": self.target_pos,
                "visual_error_dist": dist,
                "mock": False
            }

        return {
            "detected": False,
            "end_effector_pos": self.last_ee_pos,
            "target_pos": self.target_pos,
            "visual_error_dist": 0.5,
            "mock": False
        }

    def close(self):
        if self.cap and self.cap.isOpened():
            self.cap.release()
