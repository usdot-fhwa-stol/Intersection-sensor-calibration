import argparse
import csv
import os
from pathlib import Path
import cv2
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys
import copy
import skimage
from math import atan, atan2, radians, degrees, sqrt, pi, floor, cos, sin
from csv import writer
from datetime import datetime
from sklearn.preprocessing import RobustScaler
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist
from itertools import combinations

# checkerboard configuration
CHESSBOARD_SQUARE_SIZE = 0.203  # 200mm squares, measured as 203mm
CHESS_SHAPE = (3, 4)
threshold_depth = 5
THRESHOLD_ARR = np.array([10 ** threshold_depth, 10 ** threshold_depth, 10 ** threshold_depth])

# ------------------------------
# Camera & Geometry Utilities
# ------------------------------
def get_checkerboard_limits(array):
    """
    Given an array of [[x, y]] points, separately find the minimum, maximum,
     and center of both x and y
    """
    if len(np.shape(array)) == 3 and np.shape(array)[1] == 1:
        array = copy.deepcopy(array[:, 0, :])
    ix = int(np.min(array[:, 0]))
    iy = int(np.min(array[:, 1]))
    ax = int(np.max(array[:, 0]))
    ay = int(np.max(array[:, 1]))
    cx = (ax - ix) // 2 + ix
    cy = (ay - iy) // 2 + iy
    return ix, iy, ax, ay, cx, cy

def rotation_matrix_to_euler_angles(R):
    """
    Convert a rotation matrix to Euler angles (yaw, pitch, roll) in degrees.
    Assumes rotation order: XYZ (roll-pitch-yaw).
    """
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0

    return np.degrees([x, y, z])

def apply_distortion(undistorted_points, K, d):
    """
    Apply radial and tangential distortion to points using camera intrinsics.
    Input: undistorted points [[x, y]], intrinsic matrix K, distortion coefficients d
    Output: distorted points [[x_distorted, y_distorted]]
    """
    distorted_points = []
    for point in undistorted_points:
        # Compute x and y in camera coordinates
        point_image_hom = np.float32([[point[0]], [point[1]], [1]])
        point_unnorm = np.linalg.inv(K) @ point_image_hom
        x = point_unnorm[0, 0] / point_unnorm[2, 0]
        y = point_unnorm[1, 0] / point_unnorm[2, 0]

        # Apply radial distortion
        r2 = x ** 2 + y ** 2
        radial_distortion = 1 + d[0] * r2 + d[1] * r2 ** 2 + d[4] * r2 ** 3
        # Apply tangential distortion
        x_distorted = x * radial_distortion + 2 * d[2] * x * y + d[3] * (r2 + 2 * x ** 2)
        y_distorted = y * radial_distortion + d[2] * (r2 + 2 * y ** 2) + 2 * d[3] * x * y

        # Apply camera matrix
        x_distorted = x_distorted * K[0, 0] + K[0, 2]
        y_distorted = y_distorted * K[1, 1] + K[1, 2]
        distorted_points.append([[x_distorted, y_distorted]])
    return np.array(distorted_points)

def invert_transform(R, t):
    """
    Invert a rigid transform: returns R_new, t_new such that
    X_original = R_new * X_transformed + t_new
    """
    R_new = R.T
    t_new = -R_new @ t
    return R_new, t_new


# ------------------------------
# Checkerboard / Stereo Utilities
# ------------------------------
def transform_checkerboard_stereo(corners_cam2, K1, d1, K2, d2, R_cam2_cam1, tvec_cam2_cam1):
    """
    Transform checkerboard corners from camera 2 coordinates to camera 1 coordinates.
    Returns projected points in camera 1 image space with distortion applied.
    """
    # Define checkerboard coordinate system
    # Not directly used, but it is needed to compute the checkerboard coordinates in
    # the camera's frame
    checkerboard_self_coords = np.zeros((CHESS_SHAPE[0] * CHESS_SHAPE[1], 3), np.float32)
    checkerboard_self_coords[:, :2] = np.mgrid[0:CHESS_SHAPE[0], 0:CHESS_SHAPE[1]].T.reshape(-1, 2)
    checkerboard_self_coords *= CHESSBOARD_SQUARE_SIZE
    # Compute the checkerboard's transformation and 3D coordinates in camera 2
    ret, rvec_checker_cam2, tvec_checker_cam2 = cv2.solvePnP(checkerboard_self_coords, corners_cam2.reshape(-1, 2), K2,
                                                             d2)
    ###
    # an alternative method for figuring out the pose of cam2 checkerboard
    # the current method assumes corners are correct.
    # the commented out method below handles outliers better, if needed.
    ###
    # ret, rvec_checker_cam2, tvec_checker_cam2, inliers = cv2.solvePnPRansac(checkerboard_self_coords, corners_cam2.reshape(-1, 2), K2,
    #                                                          d2, reprojectionError=16)
    
    # Convert into a transformation matrix
    R_checker_cam2 = cv2.Rodrigues(rvec_checker_cam2)[0]
    T_checker_cam2 = np.eye(4)
    T_checker_cam2[0:3] = np.concatenate((R_checker_cam2, tvec_checker_cam2), axis=1)

    # Define the transformation matrix from camera 2 -> camera 1
    T_cam2_cam1 = np.concatenate((R_cam2_cam1, tvec_cam2_cam1), axis=1)

    # Compute the matrix from pixel image coordinates in camera 2 to unnormalized camera coordinates in camera 1
    # The matrices run from right to left, but the logical order is:
    #   checkerboard 3D coordinates -> coordinates in the camera frame of camera 2 (T_checker_cam2)
    #   coordinates in the camera frame of camera 2 -> coordinates in the camera frame of camera 1 (T_cam2_cam1)
    #   coordinates in the camera frame of camera 1 -> unnormalized undistorted pixel coordinates in camera 1 (K1)
    # 3x3 * 3x4 * 4x4
    mat_checker_im1 = K1 @ T_cam2_cam1 @ T_checker_cam2

    points_proj = []
    for point in checkerboard_self_coords:
        # Indirectly starting with pixel coordinates from camera 2, because we 'converted' them in the call to cv2.solvePnP above
        checker_pt = np.float32([[point[0]], [point[1]], [point[2]], [1]])

        # Compute unnormalized undistorted pixel coordinates in camera 1
        corner_im1_undis = mat_checker_im1 @ checker_pt
        # Normalize coordinates
        corner_im1_undis = (corner_im1_undis / corner_im1_undis[2])[0:2, 0].T
        # Apply camera distortion
        corner_im1 = apply_distortion([corner_im1_undis], K1, d1)[0, 0]
        points_proj.append(corner_im1)
    return np.float32(points_proj)

# K_1 | extrinsic_1 | point
# image_1 <- camera_1 | camera_1 <- checkerboard | checkerboard
def checkerboard_coords_to_im_coords(points_checker_coords, rvec, tvec, K, d):
    """
    Convert 3D checkerboard points into image coordinates with distortion applied.
    """
    R = cv2.Rodrigues(rvec)[0]
    extrinsic_matrix = np.concatenate((R, tvec), axis=1)
    P = K @ extrinsic_matrix

    points_proj = []
    for point in points_checker_coords:
        checker_zero_pt_checker = np.float32([[point[0]], [point[1]], [point[2]], [1]])

        # P * point in homogeneous coordinates
        # Then normalize by dividing by the 3rd element
        checker_zero_pt_image = P @ checker_zero_pt_checker
        checker_zero_pt_image = (checker_zero_pt_image / checker_zero_pt_image[2])[0:2, 0].T
        checker_zero_pt_image_dis = apply_distortion([checker_zero_pt_image], K, d)[0, 0]
        points_proj.append(checker_zero_pt_image_dis)
    return np.float32(points_proj)

def get_corners_pose_in_camera_frame(frame, corners1, K1, d1, corners2, K2, d2, rvec21, tvec21):
    """
    Get corners from camera 2 broadcasted into camera 1
    and then draws them on the image from camera 1.
    Returns the frame with projected corners drawn and the transformed corners.
    """
    corners_21 = transform_checkerboard_stereo(corners2, K1, d1, K2, d2, rvec21, tvec21)
    frame_21 = cv2.drawChessboardCorners(copy.deepcopy(frame), CHESS_SHAPE, corners_21, True)

    return frame_21, corners_21


def measure_corners_error(corners_true, corners_est):
    """
    Given true and estimated corners, return the
    average pixel distance error between them.
    """

    # compute average error
    avg_err = 0
    for i in range(len(corners_true)):
        for j in range(len(corners_true[0])):
            avg_err += np.sqrt((corners_true[i][j, 0] - corners_est[i][j, 0]) ** 2 + (
                        corners_true[i][j, 1] - corners_est[i][j, 1]) ** 2)
    avg_err /= (len(corners_true) * len(corners_true[0]))

    # compute error standard deviation (not yet implemented)
    # stdev_error = np.zeros(len(corners_true))
    return avg_err


def apply_limits(x_max_lim, y_max_lim, x_center, y_center, size, x_min_lim=0, y_min_lim=0, size_y=None):
    """
    Given image dimensions and x/y widths from a center point,
    crop the limits to fit inside the image.
    """
    if size_y is None:
        size_y = size
    # Compute initial boundaries as distance from the center point
    x_max = min(x_center + size, x_max_lim)
    y_max = min(y_center + size_y, y_max_lim)
    x_min = x_max - size * 2
    y_min = y_max - size_y * 2
    # Adjust limits so they fit into the image
    if x_min < x_min_lim:
        x_min = x_min_lim
        x_max = x_min_lim + size * 2
    if y_min < y_min_lim:
        y_min = y_min_lim
        y_max = y_min_lim + size_y * 2
    return int(x_max), int(x_min), int(y_max), int(y_min)


def undistort_points(image_points, K, d):
    """
    Remove distortion for an array of image points given camera intrinsics
    This helps with resizing and opencv datatypes issues.
    """
    image_points_float = np.float32(copy.deepcopy(image_points))
    image_points_undistort = cv2.undistortPoints(image_points_float, K, d, None, K)[:, 0, :]
    return image_points_undistort

def create_collage(frames, image_points, K, d, size=50):
    """
    Create a qualitative view of teh calibration accuracy.

    Output: an image with the checkerboard superimposed repeatedly to show it
    in all positions.

    Process:  Given a list of images and points in said images.
        (1) Start the collage as the first image
        (2) for each other image, grab a view around the passed in points
        and overlay on the collage
    """
    # Define initial collage, and grab image dimensions
    width = np.shape(frames[0])[1]
    height = np.shape(frames[0])[0]
    collage = frames[0]

    # Add each subsequent image to the collage
    for i, frame in enumerate(frames):
        # Draw checkerboard corners on the image
        current_image = cv2.drawChessboardCorners(copy.deepcopy(frame), CHESS_SHAPE, image_points[i], True)
        # Safely compute x and y bounds of the checkerboard to display, being careful to not go outside of the image
        # ix in min_x, ay is max_y, cx is center_x
        ix, iy, ax, ay, cx, cy = get_checkerboard_limits(image_points[i])
        x_max, x_min, y_max, y_min = apply_limits(width, height, cx,
                                                  cy, ax - ix, size_y=ay - iy)
        # Overlay this checkerboard onto the collage
        collage[y_min:y_max, x_min:x_max] = current_image[y_min:y_max, x_min:x_max]
    return collage

def read_checkerboard_corners_from_file(database_file, load_images=True):
    """
    Read [camera, ts, image, [corners 2x12]] data from an input csv file
    """
    # Read in all data
    raw_corner_data = []
    with open(database_file, 'r') as infile:
        reader = csv.reader(infile)
        for line in reader:
            # Skip completely blank lines
            if not line:
                continue
            # ignore data without a detected checkerboard
            if bool(line[3]) == True:
                raw_corner_data.append(line)

    # Convert to [camera, ts, image, [corners 2x12]]
    corner_data = []
    for corner_row in raw_corner_data:
        # raw data is [camera, ts, filename, found] + list(corners.reshape(24))
        camera_name = corner_row[0]
        timestamp = corner_row[1]
        if load_images:
            image = cv2.imread(corner_row[2])
        else:
            image = None
        corners = np.array(corner_row[4:], dtype=float).reshape(((len(corner_row) - 4) // 2), 2)
        corner_data.append([camera_name, timestamp, image, corners])

    return corner_data


def sort_corner_data_by_timestamp(corner_data):
    """
    Given an unordered list of [camera, ts, image, [corners 2x12]] from the
    recorded data, create an array of [camera, ts, image, [corners 2x12]]
    for each camera, sorted by timestamp.
    Note: The arrays may not all have matching elements
    """
    # Build sorted list of cameras and sorted list of timestamps
    timestamps = []
    cameras = []
    for detection_array in corner_data:
        camera, timestamp = detection_array[0], detection_array[1]
        if camera not in cameras:
            cameras.append(camera)
        if timestamp not in timestamps:
            timestamps.append(timestamp)
    # make sure the timestamps and cameras are sorted
    timestamps = np.sort(np.array(timestamps))
    cameras = sorted(cameras)

    # List of Lists of data sorted by timestamp, then by camera name
    corners_sorted = []
    for timestamp in timestamps:
        # List of cameras for this timestamp
        cameras_in_ts = []
        for camera in cameras:
            for detection_array in corner_data:
                if detection_array[0] == camera and detection_array[1] == timestamp:
                    cameras_in_ts.append(detection_array)
        corners_sorted.append(cameras_in_ts)
    return corners_sorted


# Given a list of [camera, ts, image, [corners 2x12]] sorted by timestamp, then inside by camera
# Return lists of [camera, ts, image, [corners 2x12]] sorted by timestamp sorted by timestamp, for each camera specified
#   Removes images not present in both cameras
def get_camera_pair_data(camera_1, camera_2, sorted_data):
    """
    Takes a list of [camera, ts, image, [corners 2x12]] sorted by timestamp,
    then inside by camera.
    Return lists of [camera, ts, image, [corners 2x12]] sorted by timestamp sorted
    for each camera specified.
    Removes images not present in both cameras.
    """
    cam_1_data = []
    cam_2_data = []
    for row in sorted_data:
        found_1_idx = -1
        found_2_idx = -1
        for i, detection_array in enumerate(row):
            if detection_array[0] == camera_1:
                found_1_idx = i
            elif detection_array[0] == camera_2:
                found_2_idx = i
        if found_1_idx != -1 and found_2_idx != -1:
            cam_1_data.append(row[found_1_idx])
            cam_2_data.append(row[found_2_idx])
    return cam_1_data, cam_2_data


def get_camera_intrinsics(camera_name):
    """
    Provides the camera intrinsics and distortion coefficients based on camera name.
    :param camera_name: The input camera name from the command line args if provided.
    If no argument is provided, uses the default camera selection.
    :return: an array of intrinsics and distortion coefficients.
    """
    # Store the camera intrinsic matrix based on the camera name
    if camera_name == "VisualCamera1":
        K1 = np.array([[1257.6302, 0, 1036.6222], [0, 1255.9806, 470.0724],
                       [0, 0, 1]])  # Pelco32 Camera intrinsic matrix [[fx 0 cx],[0 fy cy],[0 0 1]]
        d1 = np.array((-0.0697, -0.2319, -0.0189, 0.0225,
                       0.7252))  # Pelco32 Camera distortion coefficients k-radial p-tangential (k1 k2 p1 p2 k3)
    elif camera_name == "VisualCamera2":
        K1 = np.array([[3884.8106, 0, 929.0819], [0, 3926.0929, 495.2043],
                       [0, 0, 1]])  # Pelco34 Camera intrinsic matrix [[fx 0 cx],[0 fy cy],[0 0 1]]
        d1 = np.array((0.0560, 2.1691, -8.4799e-04, 0.0062,
                       -32.2268))  # Pelco34 Camera distortion coefficients k-radial p-tangential (k1 k2 p1 p2 k3)
    elif camera_name == "VisualCamera3":
        K1 = np.array([[1445.9906, 0, 944.5713], [0, 1446.1650, 566.3098],
                       [0, 0, 1]])  # Axis29 (pointed at east int) Camera intrinsic matrix [[fx 0 cx],[0 fy cy],[0 0 1]]
        d1 = np.array((-0.4052, 0.2759, -0.0016, 0.0013,
                       -0.1409))  # Axis29 (pointed at east int) Camera distortion coefficients k-radial p-tangential (k1 k2 p1 p2 k3)
    elif camera_name == "VisualCamera4":
        K1 = np.array([[1401.228583395422, 0, 964.3025714411618], [0, 1400.236048982686, 542.6711855054570],
                       [0, 0, 1]])  # Axis30 (pointed at west int) Camera intrinsic matrix [[fx 0 cx],[0 fy cy],[0 0 1]]
        d1 = np.array([-0.354146329702084, 0.176270954639400, 0.00002859007702560896, 0.00001819988580365929,
                       -0.056793085856062])  # Axis30 (pointed at west int) Camera distortion coefficients k-radial p-tangential (k1 k2 p1 p2 k3)
    # Calibration from combined data of cameras 5, 6, and 7 using the flat metal checkerboard on 4/1/2024.
    elif camera_name == 'VisualCamera5':
        K1 = np.array([[1832.272585948, 0, 654.0625163716], [0, 1831.80752859, 313.92055280365], [0, 0, 1]])
        d1 = np.array([-0.596164169546655, 0.279739486494, 0.0012185196797141, 0.001499648097780, 0.17980875688179])
    
    elif camera_name == 'VisualCamera6':
        K1 = np.array([[1808.447055401093, 0, 669.983595262202], [0, 1809.70768114778, 313.938129601148], [0, 0, 1]])
        d1 = np.array([-0.5648126860516, 0.17179727934183, 0.0019597859947493, -0.00170154778280, 0.2614596759732675])

    elif camera_name == 'VisualCamera7':
        K1 = np.array([[1834.143437168730, 0, 643.1361389271093], [0, 1833.44869030555, 323.30805273704], [0, 0, 1]])
        d1 = np.array([-0.603780365545410, 0.3990435427777, 0.001741775069729701, -0.000361156538665, -0.2146720889147548])

    elif camera_name == 'VisualCamera8':
        K1 = np.array([[1821.1854043179248, 0, 646.00451529532222], [0, 1816.0985438939467, 323.7318340197770680], [0, 0, 1]])
        d1 = np.array([-0.5505909270915795, -0.113520303371223, 0.0016112245080695, 0.00111701450823716, 1.2725341824243046])

    elif camera_name == "ThermalCamera4":
        K1 = np.array([[794.2638, 0, 320.0463], [0, 802.4150, 281.3669],
                       [0, 0, 1]])  # Flir183 Camera intrinsic matrix [[fx 0 cx],[0 fy cy],[0 0 1]]
        d1 = np.array((-0.0126, 0.4509, 0.0102,
                       3.6467e-04))  # Flir183 Camera distortion coefficients k-radial p-tangential (k1 k2 p1 p2 k3)
    else:
        print(f'unknown camera entered: {camera_name}')
        exit()
    return K1, d1

def zoom_on_checkerboard(frame, corners, size_x=500, size_y=500):
    """
    Crop and rescale a region around a detected checkerboard.
    :param frame: the input image.
    :param corners: a numpy array of detected corners.
    :return: Cropped and rescaled image centered around the checkerboard,
            with checkerboard corners drawn.
    """
    width = np.shape(frame)[1]
    height = np.shape(frame)[0]
    ix, iy, ax, ay, cx, cy = get_checkerboard_limits(corners)
    x_max, x_min, y_max, y_min = apply_limits(width, height, cx,
                                              cy, ax - ix, size_y=ay - iy)

    frame_checkerboard = cv2.drawChessboardCorners(copy.deepcopy(frame), CHESS_SHAPE, corners, True)
    frame_cropped = frame_checkerboard[y_min:y_max, x_min:x_max]
    frame_rescaled = skimage.transform.resize(frame_cropped, (size_x, size_y), anti_aliasing=True)
    return frame_rescaled

### optional function
def user_verify_data_by_camera(camera_name, database_file, out_file):
    """
    This function displays images to
    Display
    :param camera_name:
    :param database_file:
    :param out_file:
    :return:
    """
    # unordered list of [camera, ts, image, [corners 2x12]] from the recorded data
    corner_data = read_checkerboard_corners_from_file(database_file)

    # sorted by timestamp
    timestamp_referenced_data = sort_corner_data_by_timestamp(corner_data)

    # Aligned lists of data containing pairs of images with detected checkerboard and corner pixel locations
    # [[camera_name, timestamp, image, [corners 2x12]]]
    cam_data = []
    for ts_row in timestamp_referenced_data:
        for cam_row in ts_row:
            if cam_row[0] == camera_name:
                cam_data.append(cam_row)
    timestamps = [row[1] for row in cam_data]
    cam_images = [row[2] for row in cam_data]
    cam_corners = [np.float32(row[3]) for row in cam_data]

    accepted = np.zeros(len(cam_images))
    for i in range(len(cam_images)):
        cv2.imshow(f'candidate {i}', zoom_on_checkerboard(cam_images[i], cam_corners[i]))
        k = cv2.waitKey(0)
        cv2.destroyAllWindows()

        if k == 121:
            accepted[i] = True
        else:
            # Build filename from camera + timestamp
            ts_str = datetime.fromtimestamp(float(timestamps[i])).strftime("%Y-%m-%d-%H-%M-%S_%f")
            filename = f"{camera_name}_{ts_str}.png"
            print(f" Rejected: {filename}")

    print("Verification complete.")


def compute_analyze_stereo_calibration(camera_1, camera_2, database_file, output_file,
                                       show_image=False, print_results=True,
                                       use_intrinsic_guess=False, inlier_filter=False):
    """
    Function to compute the camera-camera extrinsic calibration between two cameras.
        (1) Load data from the database csv file
        (1a) define checkerboard coordinates
        (2) call opencv cv2.stereoCalibrate function to compute the extrinsics
        (3) compute error and display calibration to the user

    :param show_image: optional flag- should the script display the collage for review.
    :param print_results: optional flag--print detailed calibration summary (default no)
    :param use_intrinsic_guess: optional flag--Allow dynamic guess for camera
    intrinsics (default, no)
    :param inlier_filter: optional flag--filter noisy corner detections (default no)

    :Returns the rotation and transformation values from calibration.
    Also returns the nubmer of images filtered by inlier option if used.
    """
    # unordered list of [camera, ts, image, [corners 2x12]] from the recorded data
    corner_data = read_checkerboard_corners_from_file(database_file, show_image)

    # sorted by timestamp
    timestamp_referenced_data = sort_corner_data_by_timestamp(corner_data)

    # Camera intrinsics
    K1, d1 = get_camera_intrinsics(camera_1)
    K2, d2 = get_camera_intrinsics(camera_2)

    # Aligned lists of data containing pairs of images with detected checkerboard and corner pixel locations
    # [[camera_name, timestamp, image, [corners 2x12]]]
    cam_1_data, cam_2_data = get_camera_pair_data(camera_1, camera_2, timestamp_referenced_data)
    cam_1_images = [row[2] for row in cam_1_data]
    cam_1_corners = [np.float32(row[3]) for row in cam_1_data]
    cam_2_images = [row[2] for row in cam_2_data]
    cam_2_corners = [np.float32(row[3]) for row in cam_2_data]

    if len(cam_1_images) <= 5:
        print(f'{len(cam_1_images)} image pairs found for {camera_1} and {camera_2}, cannot compute calibration. ')
        return None, None

    # Define checkerboard coordinate system.
    # size is in meters, flat on the checkerboard
    # Used as an intermediate coordinate system to step between the cameras at each checkerboard
    #   cam 1 -> checkerboard -> cam 2 or something similar
    checkerboard_self_coords = np.zeros((CHESS_SHAPE[0] * CHESS_SHAPE[1], 3), np.float32)
    checkerboard_self_coords[:, :2] = np.mgrid[0:CHESS_SHAPE[0], 0:CHESS_SHAPE[1]].T.reshape(-1, 2)
    checkerboard_self_coords *= CHESSBOARD_SQUARE_SIZE
    objpoints = [checkerboard_self_coords for i in range(len(cam_1_images))]

    # Optional inlier filtering (per-image)
    if inlier_filter:
        filtered_cam_1_corners = []
        filtered_cam_2_corners = []
        filtered_objpoints = []

        for c1, c2, objp in zip(cam_1_corners, cam_2_corners, objpoints):
            pts1 = np.float32(c1).reshape(-1, 1, 2)
            pts2 = np.float32(c2).reshape(-1, 1, 2)

            F, inliers = cv2.findFundamentalMat(pts1, pts2, cv2.FM_RANSAC, 1.0)

            # only keep image pairs if at least half of corners are inliers.
            # this threshhold can be adjusted if needed.
            if inliers is not None and np.sum(inliers) >= len(c1) / 2:
                filtered_cam_1_corners.append(c1)
                filtered_cam_2_corners.append(c2)
                filtered_objpoints.append(objp)

        cam_1_corners = filtered_cam_1_corners
        cam_2_corners = filtered_cam_2_corners
        objpoints = filtered_objpoints

        if len(cam_1_corners) == 0:
            print(f"No image pairs passed inlier filtering for {camera_1} <-> {camera_2}.")
            return None, None

    # Stereo calibration flags
    if use_intrinsic_guess:
        stereocalibration_flags = cv2.CALIB_USE_INTRINSIC_GUESS
    else:
        stereocalibration_flags = cv2.CALIB_FIX_INTRINSIC

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
    ret_12, CM1_12, dist1_12, CM2_12, dist2_12, R_12, t_12, E_12, F_12 = cv2.stereoCalibrate(
        objpoints, cam_1_corners, cam_2_corners,
        K1, d1, K2, d2,
        (1280, 720),
        criteria=criteria,
        flags=stereocalibration_flags
    )

    # Compute inverse transform
    R_21, t_21 = invert_transform(R_12, t_12)

    # Broadcast corners from camera 1 -> 2
    frames_12, corners_12 = [], []
    for i in range(len(cam_2_images)):
        frame, corners = get_corners_pose_in_camera_frame(
            cam_2_images[i], cam_2_corners[i], K2, d2,
            cam_1_corners[i], K1, d1, R_12, t_12
        )
        frames_12.append(frame)
        corners_12.append(corners)

    # Broadcast corners from camera 2 -> 1
    frames_21, corners_21 = [], []
    for i in range(len(cam_1_images)):
        frame, corners = get_corners_pose_in_camera_frame(
            cam_1_images[i], cam_1_corners[i], K1, d1,
            cam_2_corners[i], K2, d2, R_21, t_21
        )
        frames_21.append(frame)
        corners_21.append(corners)

    # Optional display
    if show_image:
        collage_12 = create_collage(frames_12, corners_12, K1, d1)
        cv2.imshow('12 collage', collage_12)
        collage_21 = create_collage(frames_21, corners_21, K1, d1)
        cv2.imshow('21 collage', collage_21)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    # Print mean pixel error
    #if print_results:
    error_12 = measure_corners_error(cam_2_corners, corners_12)
    error_21 = measure_corners_error(cam_1_corners, corners_21)
    print(f'{camera_1} -> {camera_2} image count: {len(cam_1_images)}, '
          f'error_12: {error_12}, error_21: {error_21}')

    retained_count = len(cam_1_images)  # images remaining after inlier filtering
    return R_12, t_12, retained_count

# ------------------------------
# command line parser
# ------------------------------
def parse_args():
    """
    parses command line arguments for the script.
    potential options include:
    --database (-c), --cameras (-d), --inlier_filter, --use_intrinsic_guess,
    --verbose, --no_image --verify_checker

    :return:
    """
    parser = argparse.ArgumentParser(
        description="Compute camera-to-camera stereo calibration from checkerboard data."
    )

    parser.add_argument(
        "-d", "--database",
        type=str,
        #default=str(Path(__file__).parents[2] / "processed_data/camera_camera/camcam_17_run0/checkerboard_results.csv"),
        default=str(Path(__file__).parents[1] / "processed_data/camera_camera/camcam_17_run0/checkerboard_results.csv"),
        help="Path to checkerboard results CSV (default: camcam_17_run0 relative path)"
    )

    parser.add_argument(
        "-c", "--cameras",
        nargs='+',
        type=str,
        default=["VisualCamera5", "VisualCamera8"],
        help="List of cameras to calibrate (requires at least 2, default: VisualCamera5 VisualCamera8)"
    )

    parser.add_argument(
        "-o", "--output",
        type=str,
        default=(Path(__file__).parents[2] / "processed_data/camera_camera/camcam_17_run0/output."),
        help="List of cameras to calibrate (requires at least 2, default: VisualCamera5 VisualCamera8)"
    )

    parser.add_argument(
        "--verify_checker",
        action="store_true",
        help="Enable manual verification of checkerboard detections per camera before calibration."
    )

    parser.add_argument(
        "--inlier_filter",
        action="store_true",
        help="Filter corner points using inliers (RANSAC) before calibration"
    )

    parser.add_argument(
        "--use_intrinsic_guess",
        action="store_true",
        help="Use cv2.CALIB_USE_INTRINSIC_GUESS instead of the default CALIB_FIX_INTRINSIC"
    )

    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print detailed calibration output"
    )

    parser.add_argument(
        "--no_image",
        action="store_true",
        help="Disable displaying calibration images (enabled by default)."
    )

    args = parser.parse_args()

    if len(args.cameras) < 2:
        parser.error("You must specify at least two cameras for calibration.")

    return args



def main():
    # Parse command line arguments
    args = parse_args()

    # Resolve database path
    database_file = Path(args.database).resolve()
    print(f"Using database file: {database_file}")

    # Tuning options
    #enable_roi = args.enable_roi
    verbose = args.verbose
    show_image = not args.no_image
    #if enable_roi:
    #    print("ROI detection enabled.")
    if verbose:
        print("Verbose mode enabled.")

    # Prepare output folder
    output_folder = Path(__file__).parents[2]

    # (Optional) User verification step
    if args.verify_checker:
        print("\n--- Manual checkerboard verification enabled ---")
        for cam in args.cameras:
            output_file = output_folder / f"user_verified_{cam}_data.csv"
            user_verify_data_by_camera(cam, database_file, output_file)

    # Store all transforms
    transforms = []

    # Loop over all unique camera pairs
    for cam1, cam2 in combinations(args.cameras, 2):
        output_file = output_folder / f"camera_camera_{cam1}_{cam2}_results.csv"
        print(f"\nCalibrating camera pair: {cam1} <-> {cam2}")

        R, t, retained_count = compute_analyze_stereo_calibration(
            cam1,
            cam2,
            database_file,
            output_file,
            show_image=show_image,
            #print_results=verbose,
            use_intrinsic_guess=args.use_intrinsic_guess,
            inlier_filter=args.inlier_filter
        )

        if R is not None:
            print(f"Inlier-filtered images retained: {retained_count}")
            transforms.append({
                "pair": (cam1, cam2),
                "R": R,
                "t": t
            })

    # Print summary table
    if verbose:
        print("\nCalibration summary:")
        for tr in transforms:
            cam1, cam2 = tr["pair"]
            R = tr["R"]
            t = tr["t"]
            euler_angles = rotation_matrix_to_euler_angles(R)

            print(f"{cam1} -> {cam2}:")
            print(f"  R =\n{tr['R']}")
            print(f"  t = {tr['t']}\n")
            print(f"  Rotation (Euler angles, degrees) = {euler_angles}")


if __name__ == "__main__":
    main()
