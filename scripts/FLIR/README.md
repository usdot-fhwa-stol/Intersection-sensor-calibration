# FLIR Calibration MATLAB Scripts
This folder contains a simple MATLAB utilities for preprocessing thermal camera images from the FLIR dataset. It is designed to support calibration workflows by enhancing images before further analysis.

## requirements

- The following script assumes input files are .png
- MATLAB image processing toolbox

## Enhancement 1- Histogram Equalization
Improves contrast in FLIR images using histogram equalization.  This helps normalize the brightness levels to make features more visible between different frames.

## Enhancement 2- Sharpen Image with Median Filtering
applies median filtering with multiple kernel sizes to reduce noise, and sharpen key features in a FLIR image.

## Usage
To use just edit the configuration section at the top of the script including:
- inputDir: directory containing your thermal images (must be saved saved as.png files).
- outputDir: directory where processed images will be saved.
- medianFilterSize: Currently set to the default of 3.

### Folder Setup
The script expects the following folder structure.
```
project-root/
├─ sharpenEqualizeImages.m
├─ data/
│   ├─ input/      # Place your original .png thermal images here
│   └─ output/     # Processed images will be saved here automatically
```

### sample data

Sample FLIR images for testing have been included in the input folder.

**How it works**

The script works by reading all .png images in the specified directory, applies histogram equalization to each, applies median filtering using the built in imFilter function with the number of kernel sizes you've selected. Then saves the processed images to with your specified suffix to the output directory.