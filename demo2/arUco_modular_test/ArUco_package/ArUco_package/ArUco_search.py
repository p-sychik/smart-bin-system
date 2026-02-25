import cv2
import numpy as np

# ArUco setup - works with older OpenCV
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# Config
MARKER_SIZE_M = 0.10
STOP_DISTANCE_M = 0.6
FOCAL_LENGTH = 500


def detect(frame):
    """
    Detect ArUco markers in frame.
    Returns: (found, marker_id, distance, x_offset) or (False, None, None, None)
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Old API - no ArucoDetector class
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict)
    
    if ids is None or len(ids) == 0:
        return False, None, None, None
    
    # Get first marker found
    marker_id = int(ids[0][0])
    marker_corners = corners[0][0]
    
    # Estimate distance from marker size in pixels
    marker_width_pixels = np.linalg.norm(marker_corners[0] - marker_corners[1])
    distance = (MARKER_SIZE_M * FOCAL_LENGTH) / marker_width_pixels
    
    # Calculate x offset (-1 to 1)
    frame_center_x = frame.shape[1] / 2
    marker_center_x = np.mean(marker_corners[:, 0])
    x_offset = (marker_center_x - frame_center_x) / frame_center_x
    
    return True, marker_id, distance, x_offset


def is_aligned(x_offset, threshold=1):
    return abs(x_offset) < threshold


def is_close_enough(distance):
    return distance <= STOP_DISTANCE_M