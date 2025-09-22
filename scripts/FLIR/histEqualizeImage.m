%Read in input image and enhance contrast with histogram equalization

%% ================= Configuration =================
inputDir  = "data/input";     % dir with your png files
% example dir
% "/your/file/path/ThermalCamera4/flir-183/)"

outputDir = "data/output";    % output directory for processed files
suffix    = "_equalized";     % processed image suffix
%% =================================================
clc; clear all; close all;


%checking for output dir
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

allPngs = dir(fullfile(inputDir, '*.png'));

for i=1:numel(allPngs)
    file = allPngs(i).name;
    [~, fileName, ~] = fileparts(file);

    inputFilename  = fullfile(inputDir, file);
    outputFilename = fullfile(outputDir, strcat(fileName, suffix, ".png"));

    %Read file 
    inputImg = imread(inputFilename);

    %Perform histogram equalization
    outputImg = histeq(inputImg);

    %Save file
    imwrite(outputImg, outputFilename, 'png');
end