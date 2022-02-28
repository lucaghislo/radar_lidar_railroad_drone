function isGhost = helperClassifyGhostsUsingTracks(dynamicPosEgo, sensorPosEgo, egoCar, confirmedTracks)
% This is a helper function and may be removed or modified in a future
% release. 

% Copyright 2020 The MathWorks, Inc.

% This function checks if positions of detections defined by dynamicPosEgo
% and captured by sensors at sensorPosEgo are occluded behind tracks. 

% Ellipse parameters
[a, b, theta, xc, yc] = parseTracks(egoCar, confirmedTracks);

% Line segment starting from sensor to detection
x1 = sensorPosEgo(1,:);
y1 = sensorPosEgo(2,:);
x2 = dynamicPosEgo(1,:);
y2 = dynamicPosEgo(2,:);

% Test intersection between ellipse and line segments
tf = testIntersection(x1, y1, x2, y2, a, b, theta, xc, yc);

% Ghost are detections whose line of sight intersect with the track ellipse
% twice i.e. they are occluded behind tracks. 
isGhost = any(tf,1)';

end

% Parse tracks to return ellipse information
function [a, b, theta, xc, yc] = parseTracks(egoCar, tracks)  
    x0 = egoCar.Position(1:2)';
    phi = egoCar.Yaw;
    rot = [cosd(phi), sind(phi); -sind(phi), cosd(phi)];

    a = zeros(numel(tracks),1);
    b = zeros(numel(tracks),1);
    theta = zeros(numel(tracks),1);
    xc = zeros(numel(tracks),1);
    yc = zeros(numel(tracks),1);

    for i = 1:numel(tracks)
        trackPos = tracks(i).State([1 3]);
        trackVel = tracks(i).State([2 4]);
        theta(i) = atan2d(trackVel(2),trackVel(1));
        xyc = rot*(trackPos - x0);
        xc(i) = xyc(1);
        yc(i) = xyc(2);
        [v,d] = eig(tracks(i).Extent);
        ypr = eulerd(quaternion(blkdiag(v,1),'rotmat','point'),'ZYX','point');
        theta(i) = ypr(1) - phi;
        dims = diag(d);
        a(i) = sqrt(max(dims));
        b(i) = sqrt(min(dims));
    end
end

% Test intersection of line segments and ellipses

function tf = testIntersection(x1, y1, x2, y2, a, b, theta, xc, yc)
% Equation used for ellipse x^2/a^2 + y^2/b^2 = 1;

tf = false(numel(a),numel(x1));

% Test intersection with each ellipse. 
for i = 1:numel(a)
   % Bring line segment to ellipse frame
   R = [cosd(theta(i)) -sind(theta(i));sind(theta(i)) cosd(theta(i))];
   xy1 = [x1;y1];
   xy2 = [x2;y2];
   xye1 = R'*(xy1 - [xc(i);yc(i)]);
   xye2 = R'*(xy2 - [xc(i);yc(i)]);
   x1e = xye1(1,:);
   x2e = xye2(1,:);
   y1e = xye1(2,:);
   y2e = xye2(2,:);
   ai = a(i);
   bi = b(i);
   x21 = x2e - x1e;
   y21 = y2e - y1e;
   A = x21.^2/ai^2 + y21.^2./bi^2;
   B = 2*x1e.*x21./ai^2 + 2*y1e.*y21./bi^2;
   C = x1e.^2/ai^2 + y1e.^2/bi^2 - 1;
   D = B.^2 - 4.*A.*C;
   t1 = (-B + sqrt(D))./(2.*A);
   t2 = (-B - sqrt(D))./(2.*A);
   % Intersecting twice means occluded and declared ghost
   tf(i,:) = D > 0 & t1 > 0 & t1 < 1 & t2 > 0 & t2 < 1;
end

end