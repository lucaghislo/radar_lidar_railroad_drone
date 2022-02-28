function isGhost = helperClassifyGhostsUsingReflectors(detectionsPosEgo, sensorPosEgo, reflectors)
% This is a helper function and may be removed or modified in a future
% release.

% Copyright 2020 The MathWorks, Inc.

% A helper function to classify each dynamic detection as "ghost" or "true"
% target based on the reflectors positions in the ego coordinates. 

% Number of reflectors
k = size(reflectors,3);

% Line Segment 1 (Detections)

% Start point
x1 = sensorPosEgo(1,:);
y1 = sensorPosEgo(2,:);

% End point
x2 = detectionsPosEgo(1,:);
y2 = detectionsPosEgo(2,:);

% Line Segment 2 (Reflectors)

% Start point
x3 = reshape(reflectors(1,1,:),1,k);
y3 = reshape(reflectors(2,1,:),1,k);

% End point
x4 = reshape(reflectors(1,2,:),1,k);
y4 = reshape(reflectors(2,2,:),1,k);

tf = testIntersection(x1,y1,x2,y2,x3,y3,x4,y4);

isGhost = any(tf,2);

end

function tf = testIntersection(x1,y1,x2,y2,x3,y3,x4,y4)

x21 = x2(:) - x1(:);
y21 = y2(:) - y1(:);

y31 = y3(:)' - y1(:);
x31 = x3(:)' - x1(:);

x34 = x3(:)' - x4(:)';
y34 = y3(:)' - y4(:)';

t = (x21.*y31 - y21.*x31)./(y34.*x21 - y21.*x34);
s = (x31 - x34.*t)./x21;

% 1.1 covers the case when a detection just barely misses the reflector.
tf = t > 0 & t < 1.1 & s > 0 & s < 1.1;

end