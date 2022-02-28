function reflectors = helperFindStaticReflectors(staticPosEgo)
% This is a helper function and may be removed or modified in a future
% release. 

% Copyright 2020 The MathWorks, Inc.

% This function finds reflectors in the static detections by using their
% positions in the ego coordinate frame. 
%
% reflectors is a 2-by-2-by-N matrix, where each page represents a 2-D line
% segment connecting point (x1,y1) to (x2,y2) as [x1 x2;y1 y2].
%
persistent clusterRadarData

if isempty(clusterRadarData)    
    clusterRadarData = clusterDBSCAN('MinNumPoints',8,'Epsilon',[8 2 1]);
end

% Cluster with DBSCAN
idx = clusterRadarData(staticPosEgo');

% Remove noise points
staticPosEgo(:,idx == -1) = [];
idx(idx == -1) = [];

% Fit a line to each reflector
uqIdx = unique(idx);
numReflectors = numel(uqIdx);
m = zeros(numReflectors,1);
b = zeros(numReflectors,1);
reflectors = zeros(2,2,numReflectors);

% Fit a line for each reflector
for i = 1:numReflectors
    inliers = staticPosEgo(:,idx == uqIdx(i));
    x = inliers(1,:);
    y = inliers(2,:);
    A = [x(:) ones(numel(x),1)];
    B = y(:);
    P = A\B;
    m(i) = P(1);
    b(i) = P(2);
    x1 = min(x);
    x2 = max(x);
    y1 = m(i)*x1 + b(i);
    y2 = m(i)*x2 + b(i);
    reflectors(:,1,i) = [x1;y1];
    reflectors(:,2,i) = [x2;y2];
end