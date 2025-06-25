import cv2
import numpy as np
from ArucoDetection_definitions import getMarkerCoordinates, draw_field, four_point_transform, draw_corners, getMarkerCenter_foam, draw_numbers

# === CONFIG ===
IMAGE_PATH = "marc\src\stuff\ws.jpg"  # Change this to your input image
DESIRED_ARUCO_DICT1 = "DICT_ARUCO_ORIGINAL"
DESIRED_ARUCO_DICT2 = "DICT_6X6_50"
MARKER_LOCATION_HOLD = True

# === Define ArUco Dictionaries ===
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

# === Load Image ===
frame = cv2.imread(IMAGE_PATH)
if frame is None:
    raise FileNotFoundError(f"Could not read image: {IMAGE_PATH}")
frame_clean = frame.copy()

# === Get ArUco Detector ===
dict1 = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[DESIRED_ARUCO_DICT1])
params1 = cv2.aruco.DetectorParameters()
dict2 = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[DESIRED_ARUCO_DICT2])
params2 = cv2.aruco.DetectorParameters()

# === Detect Field Markers (dict1) ===
bboxs1, ids1, _ = cv2.aruco.detectMarkers(frame, dict1, parameters=params1)
ids_sorted1 = [id_[0] for id_ in ids1] if ids1 is not None else None
left_corners1, corner_ids1 = getMarkerCoordinates(bboxs1, ids_sorted1, 0)

# === Draw Field & Perspective Transform ===
frame_with_square, square_found = draw_field(frame, left_corners1, corner_ids1)
square_points = left_corners1 if square_found else [[10, 400], [400, 400], [400, 10], [10, 10]]
img_wrapped = four_point_transform(frame_clean, np.array(square_points))

# === Detect Object Markers (dict2) on Wrapped Image ===
bboxs2, ids2, _ = cv2.aruco.detectMarkers(img_wrapped, dict2, parameters=params2)
ids_sorted2 = [id_[0] for id_ in ids2] if ids2 is not None else None
left_corners2, corner_ids2 = getMarkerCoordinates(bboxs2, ids_sorted2, 0)
center_corner = getMarkerCenter_foam(bboxs2)

# === Draw Final Visualization ===
draw_corners(img_wrapped, center_corner)
img_wrapped = cv2.line(img_wrapped, (0, img_wrapped.shape[0]), (img_wrapped.shape[1], img_wrapped.shape[0]), (0, 0, 255), 3)
img_wrapped = cv2.line(img_wrapped, (img_wrapped.shape[1]//2, 0), (img_wrapped.shape[1]//2, img_wrapped.shape[0]), (255, 0, 0), 2)
draw_numbers(img_wrapped, left_corners2, corner_ids2)

# === Show and Save ===
cv2.imshow("Original", frame_with_square)
cv2.imshow("Warped View", img_wrapped)
cv2.imwrite("output_wrapped.jpg", img_wrapped)
cv2.waitKey(0)
cv2.destroyAllWindows()

# === Output Center ===
if center_corner:
    print("[INFO] Detected object center:", center_corner[0])
else:
    print("[INFO] No foam marker detected.")
