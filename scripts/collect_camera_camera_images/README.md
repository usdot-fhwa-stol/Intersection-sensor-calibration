# collect_camera_camera_images README

## Summary
This script allows users to capture live images and optionally check for checkerboards and store results.

---
## Usage

The script works out-of-the-box with default settings but can be customized via command-line arguments.

**Default**

If you run the script without arguments:

All available cameras in CAMERA_CONFIG will be used.

Processed images and CSV will be stored in ./processed_data (relative to the script).

You can capture live frames continuously; press Ctrl+C to stop.

After capturing frames, press y to save images. Any other key will skip storing them.

**Passing Custom Args**

You can pass custom arguments for camera selection, output folder, or to analyze stored images:
```bash
# Capture from specific cameras and store results
python stored_camera_camera_images.py --cameras VisualCamera4 VisualCamera5 --output ./my/output/folder

# Capture and then analyze all stored images in the output folder
python stored_camera_camera_images.py --analyze-stored
```

### Arguments

| Argument| Description| Default|
|----|----|---|
| `--cameras`        | List of camera names to use| All cameras in `CAMERA_CONFIG` |
| `--output`| Folder to store captured images and CSV| `./processed_data`|
| `--analyze-stored`| Flag to analyze all stored images for checkerboards after capture | No Analysis|


---
## Requirements

- Python 3.9+  

**Install dependencies:**

```bash
pip install opencv-python numpy scikit-image
```
---

## Sample Data
Here is an example of a raw image.

![Raw Image](images/raw_image.png)

---

## Example Output
here is an example of the paired output images with checkerboards drawn.
![Combined Output](images/combined.png)

csv includes:
|Camera name|Timestamp|Image Path|Checkerboard found (True/False)|Flattened corner coordinates (if found)|
|---|---|---|---|---|
|VisualCamera5|1715969042|your/png/path.png|True|Corners|
---
## Troubleshooting
- The script requires filenames to follow the format <CameraName>_YYYY-MM-DD-HH-MM-SS_microseconds.png
- users must press "y" when viewing images for images to be stored, any other key will skip storign.