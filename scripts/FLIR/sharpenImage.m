clear all; clc; close all;

%Median Filter Section
inputImg = imread('flir180.png');

%Iterate with filter size of 3,5,7
for filterSize=3:2:7
    %Filter the image with builtin imfilter function
    averageFilteredImage = medfilt2(inputImg, [filterSize filterSize], "symmetric");
    outputFilename = "flir180_" + filterSize + "x" + filterSize + "_medianfilter.png";
    imwrite(averageFilteredImage,outputFilename, 'png');
end