function dist = helperGOSPADistance(track, truth)
% This is a helper function and may be removed in a future release
% 
% This function calculates the base distance between a track and a truth
% object for GOSPA calculation.

% Copyright 2020-2021 The MathWorks, Inc.

% Calculate the actual values of the states estimated by the tracker

% Center is different than origin and the trackers estimate the center
rOriginToCenter = -truth.OriginOffset(:) + [0;0;truth.Height/2];
rot = quaternion([truth.Yaw truth.Pitch truth.Roll],'eulerd','ZYX','frame');
actPos = truth.Position(:) + rotatepoint(rot,rOriginToCenter')';
actPos = actPos(1:2); % Only 2-D

% Actual speed
actVel = truth.Velocity(1:2)';

% Actual dimensions. Only 2-D for radar
actDim = [truth.Length;truth.Width];

% Calculate error in each estimate weighted by the "requirements" of the
% system. The distance specified using Mahalanobis distance in each aspect
% of the estimate, where covariance is defined by the "requirements". This
% helps to avoid skewed distances when tracks under/over report their
% uncertainty because of inaccuracies in state/measurement models.

% Positional error
estPos = track.State([1 3]);
reqPosCov = 2*eye(2);
e = estPos - actPos;
d1 = sqrt(e'/reqPosCov*e);

% Speed error
estVel = track.State([2 4]);
reqVelCov = 5*eye(2);
e = estVel - actVel;
d2 = sqrt(e'/reqVelCov*e);

% Dimension error
e = eig(track.Extent);
dims = 2*sqrt(e)/sqrt(2); % Fitting rectangle inside ellipse.
estDim = [max(dims);min(dims)];
reqDimCov = eye(2);
e = estDim - actDim;
d3 = sqrt(e'/reqDimCov*e);

% Total distance
dist = d1 + d2 + d3;

end
