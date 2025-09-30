
clc; clear all; close all;
%% ================= Configuration =================
inputDir  = "data/input";     % dir with your png files
outputDir = "data/output";    % output directory for processed files
medianFilterSize = 3;
%% =================================================

prepareThermalImages(inputDir, outputDir, medianFilterSize);

function prepareThermalImages(inputDir, outputDir, filterSize)
    suffix = "_sharpenedEqualized";     % processed image suffix
    
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
        if length(size(inputImg)) == 3 && size(inputImg, 3) == 3
            inputImg = im2gray(inputImg);
        end
    
        %Perform histogram equalization
        outputImgStage1 = histeq(inputImg);
        %Filter the image with builtin imfilter function
        outputImg = medfilt2(outputImgStage1, [filterSize filterSize], "symmetric");
    
        %Save file
        imwrite(outputImg, outputFilename, 'png');
    end
end