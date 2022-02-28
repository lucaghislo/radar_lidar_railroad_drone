pretrainedNetURL = 'https://ssd.mathworks.com/supportfiles/lidar/data/trainedPointPillars.zip';
preTrainedMATFile = fullfile(tempdir,'trainedPointPillarsNet.mat');
preTrainedZipFile = fullfile(tempdir,'trainedPointPillars.zip');
    
if ~exist(preTrainedMATFile,'file')
    if ~exist(preTrainedZipFile,'file')
        disp('Downloading pretrained detector (8.3 MB)...');
        websave(preTrainedZipFile, pretrainedNetURL);
    end
    unzip(preTrainedZipFile, tempdir);   
end

% Add folder to path.
%addpath(fullfile(matlabroot,'examples','deeplearning_shared','main'));

% Load the pretrained network.
pretrainedNet = load(preTrainedMATFile);

% Load a point cloud.
%ptCloud = pcread(fullfile(toolboxdir('lidar'),'lidardata','highwayScene.pcd'));
ptCloud = pcread("driving_scenario_PC.ply");

% Anchor boxes.
anchorBoxes = {{3.9, 1.6, 1.56, -3.6, 0}, {3.9, 1.6, 1.56, -3.6, pi/2}};

% Cropping parameters.
gridParams = {{0.0,-39.68,-5.0},{69.12,39.68,5.0},{0.16,0.16,2.0},{432,496}};

% Label name for detected object.
classNames = {'train'};

% Confidence threshold. 
confidenceThreshold = 0.45;

% Overlap threshold. 
overlapThreshold = 0.1;

% Number of prominent pillars.
P = 12000;

% Number of points per pillar.
N = 100;

% Set the execution environment.
executionEnvironment = "auto";

% Crop the front view of the point cloud. 
processedPointCloud = helperCropFrontViewFromLidarData(ptCloud, gridParams);

% Detect the bounding boxes.
[box, ~, ~] = generatePointPillarDetections(pretrainedNet.net, processedPointCloud, ...
                      anchorBoxes, gridParams, classNames, confidenceThreshold, ...
                      overlapThreshold, P, N, executionEnvironment);

% Display the detections on the point cloud.
figure
ax = pcshow(processedPointCloud.Location);
showShape('cuboid',box,'Parent',ax,'Opacity',0.1,'Color','red','LineWidth',0.5)
hold on
zoom(ax, 1.5)
title("Detected Vehicle on Point Cloud")

function processedData = helperCropFrontViewFromLidarData(ptCloud, gridParams)
% Function to crop the front view of the point clouds
   % Set the limits for the point cloud.
   [row, column] = find(ptCloud.Location(:,:,1) < gridParams{1,2}{1} ...
       & ptCloud.Location(:,:,1) > gridParams{1,1}{1} ...
       & ptCloud.Location(:,:,2) < gridParams{1,2}{2} ...
       & ptCloud.Location(:,:,2) > gridParams{1,1}{2} ...
       & ptCloud.Location(:,:,3) < gridParams{1,2}{3} ...
       & ptCloud.Location(:,:,3) > gridParams{1,1}{3});
   ptCloud = select(ptCloud, row, column, 'OutputSize', 'full');
   finalPC = removeInvalidPoints(ptCloud);
   processedData = finalPC;           
end