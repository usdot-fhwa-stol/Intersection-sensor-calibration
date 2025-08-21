import argparse
import cv2
import numpy as np
from datetime import datetime
import skimage
import os
import copy
import csv

# ------------------------------
# Camera Configuration
# ------------------------------
CAMERA_CONFIG = {
    "VisualCamera4": ("axis", "192.168.55.30"),
    "VisualCamera5": ("radar", "192.168.55.44:51554"),
    "VisualCamera6": ("radar", "192.168.55.44:52554"),
    "VisualCamera7": ("radar", "192.168.55.44:53554"),
    "VisualCamera8": ("radar", "192.168.55.44:54554"),
}

# ------------------------------
# Helper Functions
# ------------------------------
def get_checkerboard_limits(array):
    """Find bounding box coordinates for checkerboard points."""
    ix = int(np.min(array[:, 1]))
    iy = int(np.min(array[:, 0]))
    ax = int(np.max(array[:, 1]))
    ay = int(np.max(array[:, 0]))
    cx = (ax - ix) // 2 + ix
    cy = (ay - iy) // 2 + iy
    return ix, iy, ax, ay, cx, cy

def resize_checkerboards(frame_arr, corners_arr, found_arr, size_x=500, size_y=500):
    """Crop and resize frames based on detected checkerboards."""
    resized_arr = np.zeros((len(frame_arr), size_y, size_x, 3))
    for i, frame in enumerate(frame_arr):
        if found_arr[i]:
            ix, iy, ax, ay, cx, cy = get_checkerboard_limits(corners_arr[i][:, 0])
            frame_cropped = frame[ix-50:ax+50, iy-50:ay+50]
            #cv2.imshow('0', frame_cropped)
            #cv2.waitKey(0)
            try:
                resized_arr[i] = skimage.transform.resize(frame_cropped, (size_x, size_y), anti_aliasing=True)
            except ValueError:
                resized_arr[i] = skimage.transform.resize(frame, (size_x, size_y), anti_aliasing=True)
            #cv2.imshow('0', resized_arr[i])
            #cv2.waitKey(0)
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
        print('unsupported number of cameras')
        exit()
    ret = cv2.waitKey(0)
    cv2.destroyAllWindows()
    return ret

def find_checkerboard(img, CHESS_SHAPE):
    """Detect checkerboard in image and return annotated frame, corners, found flag."""
    img_bw = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    find_flags = cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_ADAPTIVE_THRESH
    ret, corners = cv2.findChessboardCorners(img_bw, CHESS_SHAPE, None, flags=find_flags)

    if not ret:
        return img, corners, False

    img_new = cv2.drawChessboardCorners(img, CHESS_SHAPE, corners, True)
    return img_new, corners, True


def record_camera_pngs(name):
    if name not in CAMERA_CONFIG:
        print(f"Unknown camera name: {name}, exiting.")
        exit()
    device_type, ipaddr = CAMERA_CONFIG[name]


    if device_type == 'pelco':
        rtsp_url = f'rtsp://{ipaddr}/stream1'
    elif device_type == 'axis':
        rtsp_url = f'rtsp://{ipaddr}/axis-media/media.amp'
    elif device_type == 'radar':
        rtsp_url = f'rtsp://{ipaddr}/stream'
    else:  # device_type == 'flir'
        rtsp_url = f'rtsp://{ipaddr}'

    cam = cv2.VideoCapture(rtsp_url)
    ret, frame = cam.read()
    cam.release()
    if not ret:
        raise RuntimeError(f"No frame captured from {name} ({rtsp_url})")
    return frame

def check_and_store_images(frame_arr, frame_arr2, name_arr, output_path):
    """Review frames, press 'y' to store processed images in output_path."""
    board_shape = (3, 4)
    ts = datetime.now().strftime("%Y-%m-%d-%H-%M-%S_%f")
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
        os.makedirs(output_path, exist_ok=True)
        for i, frame in enumerate(frame_arr2):
            out_file = os.path.join(output_path, f"{name_arr[i]}_{ts}.png")
            cv2.imwrite(out_file, frame)
        print("Images saved.")
    else:
        print("No images stored.")


def get_camera_ts_info_from_filename(filename):
    """Return path, camera name, timestamp from filename."""
    path = '/'.join(filename.split('/')[:-1])
    non_path_filename = filename.split('/')[-1]
    camera = non_path_filename.split('_')[0]
    ts_raw = '_'.join(non_path_filename.split('_')[1:])[:-4]
    ts = datetime.strptime(ts_raw, "%Y-%m-%d-%H-%M-%S_%f").timestamp()
    return path, camera, ts

# ------------------------------
# Optional Post-Capture Analysis
# ------------------------------

def write_checkerboard_rows(output_filename, filenames, images=None, found=None, corners=None):
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
            ts_str = os.path.basename(filename).split("_", 1)[1].split(".")[0]
            camera = os.path.basename(filename).split("_")[0]
            ts = datetime.strptime(ts_str, "%Y-%m-%d-%H-%M-%S_%f").timestamp()
            if found[i]:
                writer.writerow([camera, ts, filename, found[i]] + list(corners[i].reshape(24)))
            else:
                writer.writerow([camera, ts, filename, found[i]])

def find_checkerboard_in_stored_images(folder_path):
    filenames = [os.path.join(folder_path, f) for f in os.listdir(folder_path) if f.endswith(".png")]
    # create a list of (timestamp, filename)
    files_with_ts = []
    for f in filenames:
        ts_str = os.path.basename(f).split("_", 1)[1].split(".")[0]
        ts = datetime.strptime(ts_str, "%Y-%m-%d-%H-%M-%S_%f").timestamp()
        files_with_ts.append((ts, f))
    # sort by timestamp
    files_with_ts.sort(key=lambda x: x[0])
    sorted_filenames = [f for _, f in files_with_ts]

    csv_filename = os.path.join(folder_path, "checkerboard_results.csv")
    write_checkerboard_rows(csv_filename, sorted_filenames)


# ------------------------------
# Command Line Parsing
# ------------------------------

def parse_args():
    parser = argparse.ArgumentParser(description="Process camera PNGs and find checkerboards.")
    parser.add_argument("--cameras", nargs="+", default=None, help="List of camera names (default: all available cameras")
    parser.add_argument("--output", default="./processed_data", help="Folder to store processed images and CSV (default: ./processed_data)")
    parser.add_argument("--analyze-stored", action="store_true", help="If set, analyze all stored images for checkerboards after capture")
    return parser.parse_args()

# ------------------------------
# Main
# ------------------------------

def main(args):

    name_arr = args.cameras if args.cameras else list(CAMERA_CONFIG.keys())
    output_path = args.output

    print(f"Using cameras: {name_arr}")
    print(f"Images will be saved to: {output_path}")

    try:
        while True:
            frame_arr = [record_camera_pngs(name) for name in name_arr]
            check_and_store_images(copy.deepcopy(frame_arr), copy.deepcopy(frame_arr), name_arr, output_path)
    except KeyboardInterrupt:
        print("\nLivestream capture stopped by user.")

    if args.analyze_stored:
        print("Analyzing stored frames...")
        find_checkerboard_in_stored_images(output_path)

if __name__ == '__main__':
    args = parse_args()
    main(args)