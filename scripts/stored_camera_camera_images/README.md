# stored_camera_camera_images.py

## Summary
This script allows users to process stored images captured from cameras check for checkerboards and store results for analysis.

---
## Usage

The script is designed to work out-of-the-box with default settings (relative paths, default cameras) but can be customized via command-line arguments.

**Default**

If you run the script without passing arguments it will sook for data in the folder sample_data/camera_camera/camcam_17_run0 folder (relative to the script) and will process cameras, "VisualCamera8" and "VisualCamera5".

**Passing Custom Args**

You can pass custom arguments for your data location or cameras like so:
```bash
python script.py --folder ./my/data/set --cameras VisuaCamera1 VisualCamera2
```

---
## Requirements

- Python 3.9+  


**Packages/Modules:**
- import argparse
- import cv2
- import numpy as np
- from datetime import datetime
- import re
- from collections import defaultdict
- import copy
- import skimage
- import os
- import csv
- import glob

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

---
## Troubleshooting
- filenames and paths.  The script expects files in the following format.  Unexpected slashes can cause the script fo fail.
- users must press "y" when viewing images for images to be stored, any other key will mark images as invalid.