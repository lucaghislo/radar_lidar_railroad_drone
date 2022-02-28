function [targets, ghostsStatic, ghostsDynamic, static, reflectors, classificationInfo] = helperClassifyRadarDetections(detections, egoVehicle, confirmedTracks)
% This is a helper function and may be removed or modified in a future
% release.
%
% This function classifies the objectDetections into 4 main categories.
% 1. targets - These are classified as detections from real objects.
%
% 2. ghostsStatic - These detections are classied as reflections of real
% obejcts via static objects.
%
% 3. ghostsDynamic - These detections are classified as reflections of real
% objects via real objects (or tracks)
%
% 4. static - These detections are classified as originating from static
% objects.

% detections are cell array of objectDetection objects.
%
% confirmedTracks are GGIW tracks in scenario coordinate frame.
%
% egoVehicle is an object carrying the fields Position, Velocity, Yaw,
% Pitch and Roll of the ego vehicle in the global coordinate frame.

% Odometry information
vEgo = norm(egoVehicle.Velocity);
wEgo = egoVehicle.AngularVelocity(3);

% Allocate memory
n = numel(detections);
isGhostStatic = false(n,1);
isGhostDynamic = false(n,1);
isStatic = false(n,1);

% Find unique sensors in the list
if iscell(detections)
    sensorIdx = cellfun(@(x)x.SensorIndex,detections);
else
    sensorIdx = arrayfun(@(x)x.SensorIndex,detections);
end
uqSensors = unique(sensorIdx);
detectionsPosEgo = zeros(3,n);
sensorPosEgo = zeros(3,n);

% Loop over each sensor and perform doppler analysis. Also capture the
% position of the detections in the ego coordinate frame
for i = 1:numel(uqSensors)
    thisIdx = sensorIdx == uqSensors(i);
    thisDetections = detections(thisIdx);
    [thisPosEgo, thisSensorPosEgo] = calculatePositionInEgoFrame(thisDetections);
    isStatic(thisIdx) = helperClassifyStaticAndDynamic(thisPosEgo, thisDetections, vEgo, wEgo);
    detectionsPosEgo(:,thisIdx) = thisPosEgo;
    sensorPosEgo(:,thisIdx) = repmat(thisSensorPosEgo,1,sum(thisIdx));
end

% Fit reflectors using position static detections in the ego coordinate
% frame
staticPosEgo = detectionsPosEgo(:,isStatic);
reflectors = helperFindStaticReflectors(staticPosEgo);

% Now, using reflectors find the ghost targets of dynamic detections
dynamicPosEgo = detectionsPosEgo(:,~isStatic);
dynamicSensorPosEgo = sensorPosEgo(:,~isStatic);
isGhostStatic(~isStatic) = helperClassifyGhostsUsingReflectors(dynamicPosEgo, dynamicSensorPosEgo, reflectors);
isGhostDynamic(~isStatic) = helperClassifyGhostsUsingTracks(dynamicPosEgo, dynamicSensorPosEgo, egoVehicle, confirmedTracks);

% Find targets
isTarget = ~isGhostStatic & ~isGhostDynamic & ~isStatic;

targets = detections(isTarget);
ghostsStatic = detections(isGhostStatic);
ghostsDynamic = detections(isGhostDynamic);
static = detections(isStatic);

classificationInfo = zeros(numel(detections),1);
classificationInfo(isTarget) = 1;
classificationInfo(isGhostStatic) = 2;
classificationInfo(isGhostDynamic) = 3;
classificationInfo(isStatic) = 4;

end


function [posEgo, sensorPosEgo] = calculatePositionInEgoFrame(detections)

% Calculate Cartesian positions for all detections in the "sensor"
% coordinate frame
if iscell(detections)
    allDets = [detections{:}];
else
    allDets = detections;
end
meas = horzcat(allDets.Measurement);
az = meas(1,:);
r = meas(2,:);
el = zeros(1,numel(az));
[x, y, z] = sph2cart(deg2rad(az),deg2rad(el),r);
posSensor = [x;y;z];

% Transform parameters
if iscell(detections)
    sensorToEgo = detections{1}.MeasurementParameters(1);
else
    sensorToEgo = detections(1).MeasurementParameters(1);
end
R = sensorToEgo.Orientation;
T = sensorToEgo.OriginPosition;

if isfield(sensorToEgo,'IsParentToChild') && sensorToEgo.IsParentToChild
    R = R';
end

% Position in ego frame
posEgo = T + R*posSensor;

% Position of sensor in ego frame
sensorPosEgo = T;

end
