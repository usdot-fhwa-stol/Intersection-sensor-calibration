# FLIR Calibration MATLAB Scripts
This folder contains two simple MATLAB utilities for preprocessing thermal camera images from the FLIR dataset. They are designed to support calibration workflows by enhancing images before further analysis.

## requirements

- Both of the following scripts assume input files are .png
- MATLAB image processing toolbox

## histEqualize.m
**Purpose**
Improves contrast in FLIR images using histogram equalization.  This helps normalize the brightness levels to make features more visible between different frames.

**Usage**
To use just edit the configuration section at the top of the script to include:
- inputDir: directory containing your thermal images (must be saved saved as.png files).
- outputDir: directory where processed images will be saved.
- suffix: Ending to append to processed files (the default is "_equalized")

**How it works**

The script works by reading all .png images in the specified directory, applies histogram equalization to each, then saves the processed images to with your specified suffix to the output directory.

---
## sharpenImage.m

**Purpose**
applies median filtering with multiple kernel sizes to reduce noise, and sharpen key features in a FLIR image.

**Usage**
To use just edit the input filename at the top to match your target image.

The default is a .png file "flir180.png".

**How it works**

The script works by reading in your target image.  It applies filtering with kernel sizes, 3x3, 5x5, and 7x7.  It then saves each filtered result with a filename that includes the kernel size ex:  flir180_3x3_medianfilter.png.  The output is 3 images.