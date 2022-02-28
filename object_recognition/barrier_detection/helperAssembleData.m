function [detections, configurationList] = helperAssembleData(detectionList, configurationList)
% This is a helper function and may be removed or modified in a future
% release.
% 
% This function assembles the log recorded from scenario into cell array of
% objectDetection objects and cell array of sensor configuration objects.

% Initialize memory for detections
detections = cell(0,1);

% Loop through each sensor
for i = 1:numel(detectionList)
    % List of this sensor
    thisSensorList = detectionList(i);
    
    % Number of measurements = number of detections
    numDetections = size(thisSensorList.Measurements,2);
    
    % The range-rate measurements are processed with a higher measurement
    % noise to account for 2-bounce radar detections, which can be
    % mis-associated with a real track and provide it wrong velocity. The
    % partitioning function will NOT use a scaled noise, but will use the
    % real noise value to use range-rate more strongly during partitioning
    % of detections
    thisSensorList.MeasurementNoise(3,3) = 900*thisSensorList.MeasurementNoise(3,3);
    
    % Assemble attributes
    attribs = struct('TargetIndex',0,'BounceTargetIndex',0,'BouncePathIndex',0);
    
    % Create a sample detection
    sampleDetection = objectDetection(thisSensorList.Time,...
        zeros(3,1),...
        'SensorIndex',thisSensorList.SensorIndex,...
        'MeasurementNoise',thisSensorList.MeasurementNoise,...
        'MeasurementParameters',thisSensorList.MeasurementParameters,...
        'ObjectAttributes',{attribs}...
        );
    
    % Allocate cell array for detections from this sensor
    thisDetections = repmat({sampleDetection},numDetections,1);
    
    % Fill in the data per detection
    for k = 1:numDetections
        thisDetections{k}.Measurement = thisSensorList.Measurements(:,k);
        thisDetections{k}.ObjectAttributes{1}.BouncePathIndex = thisSensorList.Attributes(k,3);
        thisDetections{k}.ObjectAttributes{1}.TargetIndex = thisSensorList.Attributes(k,1);
        thisDetections{k}.ObjectAttributes{1}.BounceTargetIndex = thisSensorList.Attributes(k,2);
    end
    
    % Store in the multi-sensor log
    detections = [detections;thisDetections];
end

% The max-range is set to inf to delete tracks outside the range field of
% view. 
for i = 1:numel(configurationList)
    configurationList{i}.IsValidTime = true;
    configurationList{i}.SensorLimits(2,2) = inf;
end

end
