%Read in input image and get mean/std dev of image
clc; clear all; close all;

allPngs = dir('/home/abey/Downloads/ThermalCamera4/flir-183/*.png*');

for i=1:numel(allPngs)
    file = allPngs(i).name;
    fileName = split(file, ".");

    inputFilename = strcat("/home/abey/Downloads/ThermalCamera4/flir-183/",fileName(1), ".png");

    inputImg = imread(inputFilename);

    %Perform histogram equalization and save image
    outputImg = histeq(inputImg);
    outputFilename = strcat("/home/abey/Downloads/ThermalCamera4/flir-183/",fileName(1), "_equalized.png");
    imwrite(outputImg,outputFilename, 'png');


end