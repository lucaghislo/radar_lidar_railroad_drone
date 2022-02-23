% Select first frame 
ptCloud = sensors(40).PointClouds{1, 1};

% Visualize input point cloud
figure
pcshow(ptCloud)
title('Input Lidar Point Cloud')
axis([0 50 -15 15 -5 5])
view([-42 35])


%% Define ROI in meters
xlim = [0 100];
ylim = [-10 10];
zlim = [-10 10];
roi = [xlim ylim zlim];

% Crop point cloud using ROI
indices = findPointsInROI(ptCloud,roi);
croppedPtCloud = select(ptCloud,indices);

% Remove ground plane
maxDistance = 0.1;
referenceVector = [0 0 1];
[model,inliers,outliers] = pcfitplane(croppedPtCloud,maxDistance,referenceVector);
groundPts = select(croppedPtCloud,inliers);

figure
pcshow(groundPts)
title('Ground Plane')
view(3)

%% Visualizzazione istogramma rilevazione
histBinResolution = 0.2;
[histVal,yvals] = helperComputeHistogram(groundPts,histBinResolution);

figure
plot(yvals,histVal,'--k')
set(gca,'XDir','reverse')
hold on

[peaks,locs] = helperfindpeaks(histVal);
startYs = yvals(locs);

laneWidth = 3;
[startLanePoints,detectedPeaks] = helperInitialWindow(startYs,peaks,laneWidth);

plot(startYs,peaks,'*r')
plot(startLanePoints,detectedPeaks,'og')
legend('Histogram','Peak','Detected Peaks','Location','North')
title('Peak Detection')
hold off

%% Peak intensity detection
histBinResolution = 0.2;
[histVal,yvals] = helperComputeHistogram(groundPts,histBinResolution);

figure
plot(yvals,histVal,'--k')
set(gca,'XDir','reverse')
hold on


%% Obtain peaks
[peaks,locs] = helperfindpeaks(histVal);
startYs = yvals(locs);

laneWidth = 3;
[startLanePoints,detectedPeaks] = helperInitialWindow(startYs,peaks,laneWidth);

plot(startYs,peaks,'*r')
plot(startLanePoints,detectedPeaks,'og')
legend('Histogram','Peak','Detected Peaks','Location','North')
title('Peak Detection')
hold off


%% Helper function find peaks
function [pkHistVal,pkIdx] = helperfindpeaks(histVal)
pkIdxTemp = (1:size(histVal,2))';
histValTemp = [NaN; histVal'; NaN];
tempIdx = (1:length(histValTemp)).';

% keep only the first of any adjacent pairs of equal values (including NaN)
yFinite = ~isnan(histValTemp);
iNeq = [1; 1 + find((histValTemp(1:end-1) ~= histValTemp(2:end)) & ...
    (yFinite(1:end-1) | yFinite(2:end)))];
tempIdx = tempIdx(iNeq);

% Take the sign of the first sample derivative
s = sign(diff(histValTemp(tempIdx)));

% Find local maxima
maxIdx = 1 + find(diff(s)<0);

% Index into the original index vector without the NaN bookend
pkIdx = tempIdx(maxIdx) - 1;

% Fetch the coordinates of the peak
pkHistVal = histVal(pkIdx);
pkIdx = pkIdxTemp(pkIdx)';
end


%% Helper function computer histogram
function [histVal,yvals] = helperComputeHistogram(ptCloud,histogramBinResolution)
numBins = ceil((ptCloud.YLimits(2) - ptCloud.YLimits(1))/histogramBinResolution);
histVal = zeros(1,numBins-1);
binStartY = linspace(ptCloud.YLimits(1),ptCloud.YLimits(2),numBins);
yvals = zeros(1,numBins-1);
for i = 1:numBins-1
    roi = [-inf 15 binStartY(i) binStartY(i+1) -inf inf];
    ind = findPointsInROI(ptCloud,roi);
    subPc = select(ptCloud,ind);
    if subPc.Count
        histVal(i) = sum(subPc.Intensity);
        yvals(i) = (binStartY(i) + binStartY(i+1))/2;
    end
end
end


%% Helper function initial window
function [yval,detectedPeaks] = helperInitialWindow(yvals,peaks,laneWidth)
leftLanesIndices = yvals >= 0;
rightLanesIndices = yvals < 0;
leftLaneYs = yvals(leftLanesIndices);
rightLaneYs = yvals(rightLanesIndices);
peaksLeft = peaks(leftLanesIndices);
peaksRight = peaks(rightLanesIndices);
diff = zeros(sum(leftLanesIndices),sum(rightLanesIndices));
for i = 1:sum(leftLanesIndices)
    for j = 1:sum(rightLanesIndices)
        diff(i,j) = abs(laneWidth - (leftLaneYs(i) - rightLaneYs(j)));
    end
end
[~,minIndex] = min(diff(:));
[row,col] = ind2sub(size(diff),minIndex);
yval = [leftLaneYs(row) rightLaneYs(col)];
detectedPeaks = [peaksLeft(row) peaksRight(col)];
estimatedLaneWidth = leftLaneYs(row) - rightLaneYs(col);

% If the calculated lane width is not within the bounds,
% return the lane with highest peak
if abs(estimatedLaneWidth - laneWidth) > 0.5
    if max(peaksLeft) > max(peaksRight)
        yval = [leftLaneYs(maxLeftInd) NaN];
        detectedPeaks = [peaksLeft(maxLeftInd) NaN];
    else
        yval = [NaN rightLaneYs(maxRightInd)];
        detectedPeaks = [NaN rightLaneYs(maxRightInd)];
    end
end
end