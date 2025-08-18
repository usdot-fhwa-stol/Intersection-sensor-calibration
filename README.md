# Intersection Sensor Calibration

## Project Overview
This repository contains scripts and tools developed for the U.S. DOT Intersection Safety Challenge to perform intrinsic and extrinsic calibration of visual cameras, thermal cameras, and LiDAR sensors used in real-world intersection environments. The methods extend standard calibration techniques to handle challenges such as large distances between sensors, poor field-of-view overlap, low-resolution thermal imagery, and limited marker visibility—resulting in accurate multi-sensor alignment.

---
## Repo Structure
```bash
repo_root/
├── scripts/
|   |── camera_camera_stereo_calibrate.py
|   |── collect_camera_camera_images.py
|   |── stored_camera_camera_images
├── sample_data/
├── images/
└── README.md
```

---
## Setup Instructions

### Folder Structure and Data
Due to size sample data is not included in this repository.
You can download it here [sample_data.zip](https://data.transportation.gov/api/views/vq7s-mv3v/files/dee124ae-7422-4ab3-a08c-244c4f6133d5?download=true&filename=Calibration%20Data.zip)

After downloading, unzip it and recreate the following structure:
```bash
ISC/
├── scripts
├── sample_data/
|   └── camera_camera/
|       └── camcam_17_run0/
|           ├── VisualCamera5Example_.png
|           └── VisualCamera8Example_.png
└── processed_data
```


### Requirements

- Python 3.9+  


**Packages/Modules:**
- datetime
- timedelta (from datetime)
- copy
- os
- csv
- glob
- pathlib
- sys
- math (atan, atan2, radians, degrees, sqrt, pi, floor, cos, sin)
- opencv-python (for cv2)
- numpy
- scikit-image (for skimage)
- pandas
- matplotlib
- scikit-learn (for sklearn.preprocessing)
- scipy


**Install dependencies:**

```bash
pip install opencv-python numpy scikit-image pandas matplotlib scikit-learn scipy
```
---
## Usage

These scripts are written to be used in sequence.  First capture and perform initial processing with collect_camera_camera_images or stored_camera_camera_images.  Next perform calibration using camera_camera_stereo_calibrate.

---

## Scripts

| Script Name | Purpose |
|-------------|---------|
| `collect_camera_camera_images.py` | Captures images from camera livestreams and performs initial processing. |
| `stored_camera_camera_images.py` | performs initial processing with stored images. | 
| `camera_camera_stereo_calibrate.py` | Runs stereo calibration between two cameras. |



---

## Contribution Guide

*in progress*

## Troubleshooting
*for troubleshooting guidance please refer to each scripts readme or in-line documentaiton in the script itself.*

## License and Credits
*in progress*