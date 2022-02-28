function isStatic = helperClassifyStaticAndDynamic(posEgo, detections, speedEgo, yawRateEgo)
% This is a helper function and may be removed or modified in a future
% release. 
% 
% Copyright 2020 The MathWorks, Inc.

% A helper function to classify if detections are static or dynamic in the
% scene using reported detections and speed and yaw-rate of the ego
% vehicle. "detections" are assumed to be from the same sensor. Therefore,
% this process must be done separately for each sensor. See
% helperClassifyRadarDetections. 

% First, calculate the position of the reported detections in the ego
% coordinate frame

% Compute reported range-rate for each detection
if iscell(detections)
    allDets = [detections{:}];
else
    allDets = detections;
end
meas = horzcat(allDets.Measurement);
rr = meas(3,:);

% Compute the expected range-rate of a static target from this sensor
state = zeros(6,numel(detections));
state(1:2:end,:) = posEgo;
vEgo = [speedEgo;0;0];
state(2:2:end,:) = repmat(-vEgo(:),1,numel(detections));

if iscell(detections)
    params = detections{1}.MeasurementParameters(1);
else
    params = detections(1).MeasurementParameters(1);
end
wEgo = [0 0 deg2rad(yawRateEgo)];
params.OriginVelocity = cross(deg2rad(wEgo(:)),params.OriginPosition);

expMeas = cvmeas(state,params);
expRR = expMeas(3,:);

% Compare with reported range-rate. If similar, qualify as static target
isStatic = abs(expRR - rr) < 2 & abs(expRR) > 2;

end

