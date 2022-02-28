function partitions = helperMultipathExamplePartitionFcn(detections, varargin)
% This is a helper function and may be modified or removed in a future
% release. 
% 
% This function calculates the partitions of a detections using
% partitionDetections function. Before, it accomodates the scaled
% range-rate measurement noise as well as uses the speed (assuming parallel
% to ego vehicle) for clustering. 

% Copyright 2021 The MathWorks, Inc.

% Calculate sensor yaw angle in ego frame
ypr = eulerd(quaternion(detections{1}.MeasurementParameters(1).Orientation,'rotmat','point'),'ZYX','point');
sensorYaw = ypr(1);

% For each detection, convert range-rate to appropriate speed
for i = 1:numel(detections)
    rr = detections{i}.Measurement(3);
    r =  detections{i}.Measurement(2);
    az = detections{i}.Measurement(1);
    % Use speed parallel to ego vehicle for clustering objects close to
    % ego. 
    cosTheta = cosd(sensorYaw + az);
    detections{i}.Measurement(3) = rr./cosTheta;
    
    % is the azimuth ~= 0, if yes, don't use speed for evaluating these
    % detections by setting their range-rate measurement noise high. 
    isCloseToZero = abs(cosTheta) < 0.02;
    
    % Are detections close to side-mounted sensors? If yes, use higher
    % measurement noise on azimuth to account that a target can span 60-80
    % degrees easily and may have missed detections on the surface. 
    isCloseToSideSensors = detections{i}.SensorIndex > 2 & r < 5;
    
    % Scale range-rate noise
    if isCloseToZero
        detections{i}.MeasurementNoise(3,3) =  625;
    else
        detections{i}.MeasurementNoise(3,3) =  1/900*detections{i}.MeasurementNoise(3,3);
    end
    
    % Scale azimuth noise    
    if isCloseToSideSensors
        detections{i}.MeasurementNoise(1,1) = 100;
    end
end

% Partition using Mahalanobis distance
partitions = partitionDetections(detections,varargin{:});

end