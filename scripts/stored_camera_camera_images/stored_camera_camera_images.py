import argparse
import cv2
import numpy as np
from datetime import datetime
import re
from collections import defaultdict
import copy
import skimage
import os
import csv
import glob

# ------------------------------
# Helper Functions
# ------------------------------

def extract_ts(filename):
    """Extract timestamp string from filename."""
    match = re.search(r'\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2}_\d+', filename)
    return match.group(0) if match else None


def get_checkerboard_limits(array):
    """Find min/max/center x and y coordinates from input array."""
    ix = int(np.min(array[:, 1]))
    iy = int(np.min(array[:, 0]))
    ax = int(np.max(array[:, 1]))
    ay = int(np.max(array[:, 0]))
    cx = (ax - ix) // 2 + ix
    cy = (ay - iy) // 2 + iy
    return ix, iy, ax, ay, cx, cy


def resize_checkerboards(frame_arr, corners_arr, found_arr, size_x=500, size_y=500):
    """Crop and resize images to zoom in on detected checkerboard for easier viewing."""
    resized_arr = np.zeros((len(frame_arr), size_y, size_x, 3))
    for i, frame in enumerate(frame_arr):
        if found_arr[i]:
            ix, iy, ax, ay, cx, cy = get_checkerboard_limits(corners_arr[i][:, 0])
            try:
                frame_cropped = frame[ix - 50:ax + 50, iy - 50:ay + 50]
                resized_arr[i] = skimage.transform.resize(frame_cropped, (size_x, size_y), anti_aliasing=True)
            except ValueError:
                resized_arr[i] = skimage.transform.resize(frame, (size_x, size_y), anti_aliasing=True)
        else:
            resized_arr[i] = skimage.transform.resize(frame, (size_x, size_y), anti_aliasing=True)
    return resized_arr


def structure_camera_images(resized_img_arr, name_arr):
    """Display cameras in a grid and return key press."""
    if len(resized_img_arr) == 1:
        cv2.imshow(f'{name_arr[0]}', resized_img_arr[0])
    elif len(resized_img_arr) == 2:
        combined = np.hstack((resized_img_arr[0], resized_img_arr[1]))
        cv2.imshow('combined', combined)
    elif len(resized_img_arr) <= 4:
        combined1 = np.hstack((resized_img_arr[0], resized_img_arr[1]))
        combined2 = np.hstack((resized_img_arr[2], np.zeros(np.shape(resized_img_arr[0]))))
        combined = np.vstack((combined1, combined2))
        cv2.imshow('combined', combined)
    elif len(resized_img_arr) <= 6:
        combined1 = np.hstack((resized_img_arr[0], resized_img_arr[1], resized_img_arr[2]))
        if len(resized_img_arr) == 5:
            combined2 = np.hstack((resized_img_arr[3], resized_img_arr[4], np.zeros(np.shape(resized_img_arr[0]))))
        else:
            combined2 = np.hstack((resized_img_arr[3], resized_img_arr[4], resized_img_arr[5]))
        combined = np.vstack((combined1, combined2))
        cv2.imshow('combined', combined)
    else:
        print('Unsupported number of cameras')
        exit()
    ret = cv2.waitKey(0)
    cv2.destroyAllWindows()
    return ret


def find_checkerboard(img, CHESS_SHAPE):
    """Detect checkerboard in image and return annotated frame, corners, found flag."""
    img_bw = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_ADAPTIVE_THRESH
    ret, corners = cv2.findChessboardCorners(img_bw, CHESS_SHAPE, None, flags)
    if not ret:
        return img, corners, False
    img_new = cv2.drawChessboardCorners(img, CHESS_SHAPE, corners, True)
    return img_new, corners, True


def get_frame_from_file(filename):
    """Read image from file."""
    frame = cv2.imread(filename)
    if frame is None:
        raise FileNotFoundError(f'No image found at {filename}')
    return frame


def check_and_store_images(frame_arr, frame_arr2, name_arr, output_path):
    """Review frames, press 'y' to store processed images in output_path."""
    board_shape = (3, 4)
    ts = f'{datetime.now().strftime("%Y-%m-%d-%H-%M-%S_%f")}'
    checker_img_arr = [None] * len(frame_arr)
    corners_arr = np.zeros((len(frame_arr), board_shape[0]*board_shape[1], 1, 2))
    found_arr = np.zeros(len(frame_arr))

    for i in range(len(frame_arr)):
        checker_img_arr[i], corners_arr[i], found_arr[i] = find_checkerboard(frame_arr[i], board_shape)
        if not found_arr[i]:
            print(f'No checkerboard in camera {name_arr[i]}.')

    resized_checkers = resize_checkerboards(checker_img_arr, corners_arr, found_arr)
    ret = structure_camera_images(resized_checkers, name_arr)

    if ret == 121:  # 'y' key
        print('Writing images...')
        for i, frame in enumerate(frame_arr2):
            out_file = os.path.join(output_path, f"{name_arr[i]}_{ts}.png")
            cv2.imwrite(out_file, frame)
    else:
        print('No images stored.')


def get_filenames_filter(dir, start_filter='', end_filter=''):
    """Return list of file paths filtered by start and end string."""
    files = [os.path.join(dir, f) for f in os.listdir(dir) if os.path.isfile(os.path.join(dir, f))]
    start_len = len(os.path.join(dir, start_filter))
    matched_files = [f for f in files if f[:start_len] == os.path.join(dir, start_filter) and f.endswith(end_filter)]
    return matched_files


def sort_filenames_timestamps(filenames):
    """Sort filenames by timestamp."""
    timestamps = []
    cameras = []
    path = None
    for filename in filenames:
        path, camera, ts = get_camera_ts_info_from_filename(filename)
        if camera not in cameras:
            cameras.append(camera)
        if ts not in timestamps:
            timestamps.append(ts)
    timestamps = np.sort(np.array(timestamps))
    cameras = sorted(cameras)
    filenames_sorted = []
    for timestamp in timestamps:
        ts_datetime = datetime.fromtimestamp(timestamp)
        for camera in cameras:
            filename = f'{path}\\{camera}_{ts_datetime.strftime("%Y-%m-%d-%H-%M-%S_%f")}.png'
            if filename in filenames:
                filenames_sorted.append(filename)
    return filenames_sorted


def get_camera_ts_info_from_filename(filename):
    """Return path, camera name, timestamp from filename."""
    path = os.path.dirname(filename)
    non_path_filename = os.path.basename(filename)
    camera = non_path_filename.split('_')[0]
    ts_str = non_path_filename.split("_", 1)[1].split(".")[0]
    ts = datetime.strptime(ts_str, "%Y-%m-%d-%H-%M-%S_%f").timestamp()
    return path, camera, ts


def write_checkerboard_rows(output_filename, filenames, images=None, found=None, corners=None):
    """Write CSV with camera, timestamp, filename, checkerboard found flag, corner locations."""
    if not images:
        images = [cv2.imread(f) for f in filenames]

    if not found:
        found = []
        corners = []
        ten_percent = max(len(images) // 10, 1)
        for i, image in enumerate(images):
            if i % ten_percent == 0:
                print(f'{((i * 10) // ten_percent)}% complete')
            _, corners_tmp, found_tmp = find_checkerboard(image, (3, 4))
            found.append(found_tmp)
            corners.append(corners_tmp)

    with open(output_filename, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        for i, filename in enumerate(filenames):
            _, camera, ts = get_camera_ts_info_from_filename(filename)
            if found[i]:
                writer.writerow([camera, ts, filename, found[i]] + list(corners[i].reshape(24)))
            else:
                writer.writerow([camera, ts, filename, found[i]])


def find_checkerboard_in_stored_images(folder_path):
    """Process all images in folder and create CSV with results."""
    filenames = get_filenames_filter(folder_path, end_filter='.png')
    filenames = sort_filenames_timestamps(filenames)
    csv_filename = os.path.join(folder_path, 'checkerboard_results.csv')
    write_checkerboard_rows(csv_filename, filenames)


# ------------------------------
# Processing Pipeline
# ------------------------------

def load_pngs(folder_path):
    png_files = sorted(glob.glob(os.path.join(folder_path, "*.png")))
    print(f"Found {len(png_files)} PNG files in {folder_path}.")
    return png_files


def build_timestamp_dict(png_files, name_arr):
    ts_dict = defaultdict(dict)
    for file in png_files:
        ts = extract_ts(file)
        if ts is None:
            continue
        for name in name_arr:
            if name in os.path.basename(file):
                ts_dict[ts][name] = file
    return ts_dict


def process_images(ts_dict, name_arr, output_path):
    for ts, files in sorted(ts_dict.items()):
        missing_cameras = [name for name in name_arr if name not in files]
        if missing_cameras:
            print(f"Skipping timestamp {ts}: missing frames from {missing_cameras}")
            continue
        print(f"Processing timestamp {ts}")
        frame_arr = [get_frame_from_file(files[name]) for name in name_arr]
        check_and_store_images(copy.deepcopy(frame_arr), copy.deepcopy(frame_arr), name_arr, output_path)


# ------------------------------
# Command Line Parsing
# ------------------------------

def parse_args():
    parser = argparse.ArgumentParser(description="Process camera PNGs and find checkerboards.")
    parser.add_argument("--folder", default=None, help="Path to PNG folder (default: ./sample_data)")
    parser.add_argument("--cameras", nargs="+", default=None, help="List of camera names (default: ['VisualCamera8','VisualCamera5'])")
    parser.add_argument("--output", default=None, help="Folder to store processed images and CSV (default: ./processed_data)")
    return parser.parse_args()


# ------------------------------
# Main
# ------------------------------

def main(args):
    folder_path = args.folder or os.path.join(os.path.dirname(__file__), "sample_data", "camera_camera", "camcam_17_run0")
    output_path = args.output or os.path.join(os.path.dirname(__file__), "processed_data", "camera_camera", "camcam_17_run0")
    os.makedirs(output_path, exist_ok=True)
    name_arr = args.cameras or ["VisualCamera8", "VisualCamera5"]

    png_files = load_pngs(folder_path)
    ts_dict = build_timestamp_dict(png_files, name_arr)
    process_images(ts_dict, name_arr, output_path)

    find_checkerboard_in_stored_images(output_path)


if __name__ == '__main__':
    args = parse_args()
    main(args)
