classdef helperLidarTrackingAlgorithm  < matlab.System
    properties
        Detector
        Tracker
        StateTransitionFcn = @helperctcuboid
    end
    
    % For calculating Pd of all tracks
    properties (Access = protected)
        DetectableTracksInput;
    end
    
    methods
        function obj = helperLidarTrackingAlgorithm(lidar)
            assert(strcmpi(lidar.DetectionCoordinates,'Ego Cartesian'),...
                'This helper class only supports lidar point cloud reported in ''Ego Cartesian'' frame');
            
            obj.Tracker = trackerJPDA('FilterInitializationFcn',@helperInitIMMFilter,...
                'TrackerIndex',2,...
                'TrackLogic','History',...
                'AssignmentThreshold',[250 inf],...
                'ClutterDensity',1e-9,...
                'ConfirmationThreshold',[7 10],...
                'DeletionThreshold',[8 10],...
                'HasDetectableTrackIDsInput',true,...
                'InitializationThreshold',0,...
                'MaxNumTracks',100,...
                'HitMissThreshold',0.1);
            
            obj.Detector =  HelperBoundingBoxDetector(...
                'XLimits',[-120 120],...              % min-max
                'YLimits',[-20 20],...                % min-max
                'ZLimits',[-2 5],...                  % min-max
                'SegmentationMinDistance',1.9,...     % minimum Euclidian distance
                'MinDetectionsPerCluster',1,...       % minimum points per cluster
                'EgoVehicleRadius',0.5,...            % To remove ego point cloud
                'MeasurementNoise',blkdiag(0.05*eye(3),25,2*eye(3)),...% measurement noise in detection report
                'MeasurementParameters',measParamStruct,...% MeasurementParameters in detection report
                'GroundMaxDistance',0.3);           % maximum distance of ground points from ground plane
        end
    end
    
    methods (Access = protected)
        function setupImpl(obj, varargin)
            detInput = zeros(0,2);
            coder.varsize('detInput',[obj.Tracker.MaxNumTracks 2],[1 0]);
            obj.DetectableTracksInput = detInput;
        end
    end
    
    methods (Access = protected)
        function [lidarTracks, detections, segmentationInfo, trackerInfo] = stepImpl(obj, egoPose, ptCloud, time)
            tracker = obj.Tracker;
            
            [detections,obstacleIndices,groundIndices,croppedIndices]  = obj.Detector(ptCloud,time);
            
            [detections, egoParams] = preprocessDetections(detections, egoPose);
            
            detectableTrackIdsInput = obj.DetectableTracksInput;
            
            [lidarTracks, ~, allTracks, trackerInfo] = tracker(detections, time, detectableTrackIdsInput);
            
            obj.DetectableTracksInput = helperCalcDetectability(allTracks,[1 2 6],egoParams);
            
            segmentationInfo = struct('CroppedIndices',croppedIndices,...
                'GroundIndices',groundIndices,...
                'ObstacleIndices',obstacleIndices);
        end
    end
end

function [dets, egoMeasParams] = preprocessDetections(dets, egoPose)
% Add Ego-Vehicle's information to the detection's MeasurementParameters to
% track state in the scenario frame.

egoMeasParams = struct('Frame',drivingCoordinateFrameType(1),...
    'OriginPosition',egoPose.Position(:),...
    'OriginVelocity',egoPose.Velocity(:),...
    'Orientation', rotmat(quaternion([egoPose.Yaw, egoPose.Pitch egoPose.Roll],'eulerd','ZYX','frame'),'frame')...
    );
for i = 1:numel(dets)
    dets{i}.MeasurementParameters = egoMeasParams;
end
end

%% Helper functions
function filter = helperInitIMMFilter(detection)
% This is a helper function and may be removed in a future release.
% This function initializes an IMM filter for tracking using lidar data.

% Copyright 2019 The MathWorks, Inc.

% Create individual filters for constant turn rate and constant velocity.
trackingfilters = cell(2,1);
trackingfilters{1} = initCTCuboidFilter(detection);
trackingfilters{2} = initCVCuboidFilter(detection);

% Construct IMM and inform it about switching between models.
filter = trackingIMM(trackingfilters,'ModelConversionFcn',@switchimmCuboid);
filter.MeasurementNoise = detection.MeasurementNoise;
filter.TransitionProbabilities = 0.9;
end


function x = switchimmCuboid(modelType1,x1,modelType2,varargin)

if strcmpi(modelType1,modelType2)
    x = x1;
elseif strcmpi(modelType1,'helpercvcuboid') % Convert from cv to ct
    x = cv2ct(x1);
elseif strcmpi(modelType1,'helperctcuboid') % Convert from ct to cv
    x = ct2cv(x1);
end

end

function xct = cv2ct(xcv)
H = [eye(4) zeros(4,5);zeros(1,9);zeros(5,4) eye(5)];
if isvector(xcv)
    xct = H*xcv;
else
    xct = H*xcv*H';
    xct(5,5) = 1;
end
end

function xcv = ct2cv(xct)
H = [eye(4) zeros(4,6);zeros(5,5) eye(5)];
if isvector(xct)
    xcv = H*xct;
else
    xcv = H*xct*H';
end
end

function filter = initCVCuboidFilter(detection)
% This function initializes a constant-velocity cuboid filter from a
% detection report.
% detection contains measurement with the following convention:
% [x;y;z;l;w;h];

posIndex = [1 2 5];
velIndex = [3 6];
dimIndex = [7 8 9];
yawIndex = 4;

meas = detection.Measurement;
measCov = detection.MeasurementNoise;
params = detection.MeasurementParameters;

dataType = class(meas);
[pos,posCov,dim,dimCov,yaw,yawCov,vel,velCov] = helperInverseLidarModel(meas,measCov,params);

% Assemble state and state covariances
state = zeros(9,1,dataType);
state(posIndex) = pos;
state(dimIndex) = dim;
state(yawIndex) = yaw;
state(velIndex) = vel;
cov = eye(9,dataType);
cov(posIndex,posIndex) = posCov;
cov(dimIndex,dimIndex) = dimCov;
cov(yawIndex,yawIndex) = yawCov;
cov(velIndex,velIndex) = velCov;

% processNoise. Acceleration and omega
Q = eye(3);
Q(2,2) = 30^2/12;

% Use a UKF for capture non-linearity.
filter = trackingUKF(@helpercvcuboid,@helpercvcuboidmeas,state,...
    'StateCovariance',cov,...
    'HasAdditiveProcessNoise',false,...
    'ProcessNoise',Q,'MeasurementNoise',detection.MeasurementNoise,...
    'Alpha',0.01);

n = numel(detection.Measurement);
setMeasurementSizes(filter,n,n);

end


function filter = initCTCuboidFilter(detection)
% This function initializes a constant-turn rate cuboid filter from a
% detection report.
% detection contains measurement with the following convention:
% [x;y;z;l;w;h];

posIndex = [1 2 6];
velIndex = [3 7];
dimIndex = [8 9 10];
yawIndex = 4;

meas = detection.Measurement;
measCov = detection.MeasurementNoise;
params = detection.MeasurementParameters;
dataType = class(meas);
[pos,posCov,dim,dimCov,yaw,yawCov,vel,velCov] = helperInverseLidarModel(meas,measCov,params);

% Assemble state and state covariances
state = zeros(10,1,dataType);
state(velIndex) = vel;
state(posIndex) = pos;
state(dimIndex) = dim;
state(yawIndex) = yaw;
cov = 100*eye(10,dataType);
cov(posIndex,posIndex) = posCov;
cov(dimIndex,dimIndex) = dimCov;
cov(yawIndex,yawIndex) = yawCov;
cov(velIndex,velIndex) = velCov;
cov(5,5) = 25;

% processNoise
Q = eye(3);
Q(2,2) = 5^2/12;

% Use a UKF for capture non-linearity.
filter = trackingUKF(@helperctcuboid,@helperctcuboidmeas,state,...
    'StateCovariance',cov,...
    'HasAdditiveProcessNoise',false,...
    'ProcessNoise',Q,...
    'MeasurementNoise',detection.MeasurementNoise,...
    'Alpha',0.01);

n = numel(detection.Measurement);
setMeasurementSizes(filter,n,n);
end


function meas = helperLidarModel(pos,dim,yaw)
% This function returns the expected bounding box measurement given an
% object's position, dimension, and yaw angle.

% Copyright 2019 The MathWorks, Inc.

% Get x,y and z.
x = pos(1,:);
y = pos(2,:);
z = pos(3,:) - 2; % lidar mounted at height = 2 meters.

% Get spherical measurement.
[az,~,r] = cart2sph(x,y,z);

s  = 3/50;
sz = 2/50;

% Get length, width and height.
L = dim(1,:);
W = dim(2,:);
H = dim(3,:);

az = az - deg2rad(yaw);

% Shrink length along radial direction.
Lshrink = min(L,abs(s.*r.*(cos(az))));
Ls = L - Lshrink;

% Shrink width along radial direction.
Wshrink = min(W,abs(s.*r.*(sin(az))));
Ws = W - Wshrink;

% Shrink height.
Hshrink = min(H,sz.*r);
Hs = H - Hshrink;

% Similar shift is for x and y directions.
shiftX = Lshrink.*cosd(yaw) + Wshrink.*sind(yaw);
shiftY = Lshrink.*sind(yaw) + Wshrink.*cosd(yaw);
shiftZ = Hshrink;

% Modeling the affect of box origin offset
x = x - sign(x).*shiftX/2;
y = y - sign(y).*shiftY/2;
z = z + shiftZ/2 + 2;

% Measurement format
meas = [x;y;z;yaw;Ls;Ws;Hs];

end

function detectableTracksInput = helperCalcDetectability(tracks,posIndices,varargin)
% This is a helper function to calculate the detection probability of
% tracks for the lidar tracking example. It may be removed in a future
% release.

% Copyright 2019 The MathWorks, Inc.

% The bounding box detector has low probability of segmenting point clouds
% into bounding boxes are distances greater than 40 meters. This function
% models this effect using a state-dependent probability of detection for
% each tracker. After a maximum range, the Pd is set to a high value to
% enable deletion of track at a faster rate.
if isempty(tracks)
    detectableTracksInput = zeros(0,2);
    return;
end
rMax = 75;
rAmbig = 40;
stateSize = numel(tracks(1).State);
posSelector = zeros(3,stateSize);
posSelector(1,posIndices(1)) = 1;
posSelector(2,posIndices(2)) = 1;
posSelector(3,posIndices(3)) = 1;
pos = getTrackPositions(tracks,posSelector);

if nargin == 3
    params = varargin{1};
    pos = convertToEgo(params,pos')';
end
if coder.target('MATLAB')
    trackIDs = [tracks.TrackID];
else
    trackIDs = zeros(1,numel(tracks),'uint32');
    for i = 1:numel(tracks)
        trackIDs(i) = tracks(i).TrackID;
    end
end
[~,~,r] = cart2sph(pos(:,1),pos(:,2),pos(:,3));
probDetection = 0.9*ones(numel(tracks),1);
probDetection(r > rAmbig) = 0.4;
probDetection(r > rMax) = 0.99;
detectableTracksInput = [double(trackIDs(:)) probDetection(:)];
end


function [pos,posCov,dim,dimCov,yaw,yawCov,vel,velCov] = helperInverseLidarModel(meas,measCov,params)
% This function returns the position, dimension, yaw using a bounding
% box measurement.

% Copyright 2019 The MathWorks, Inc.

% Shrink rate.
s = 3/50;
sz = 2/50;

% x,y and z of measurement
x = meas(1,:);
y = meas(2,:);
z = meas(3,:);

[az,~,r] = cart2sph(x,y,z);

% Shift x and y position.
Lshrink = abs(s*r.*(cos(az)));
Wshrink = abs(s*r.*(sin(az)));
Hshrink = sz*r;

shiftX = Lshrink;
shiftY = Wshrink;
shiftZ = Hshrink;

x = x + sign(x).*shiftX/2;
y = y + sign(y).*shiftY/2;
z = z + sign(z).*shiftZ/2;

pos = [x;y;z];
posCov = measCov(1:3,1:3);

yaw = meas(4,:);
yawCov = measCov(4,4,:);

% Dimensions are uncertain
dim = [6;3;3];
lCov = 6^2/12;
wCov = 3^2/12;
hCov = 3^2/12;
lwCorr = 0.5;
lwCov = sqrt(lCov*wCov)*lwCorr;
dimCov = [lCov lwCov 0;lwCov wCov 0;0 0 hCov];

vel = [0;0];
velCov = diag([30^2/12,1]);

% If params not empty, transform to scenario coordinates
if ~isempty(params)
    egoPos = params.OriginPosition(:);
    egoOrient = params.Orientation;
    ypr = eulerd(quaternion(params.Orientation,'rotmat','frame'),'ZYX','frame');
    egoYaw = ypr(1);
    pos = egoOrient'*pos + egoPos;
    posCov = egoOrient'*posCov*egoOrient;
    yaw = yaw + egoYaw;
    yawCov = 45^2/12;
    vel(1) = vel(1) + norm(params.OriginVelocity); % Simply added no orientation.
end
end

function meas = helperctcuboidmeas(state, varargin)

% Convention for state is [x;y;s;theta;omega;z;vz;L;W;H];
pos = state([1 2 6],:);
dim = state([8 9 10],:);
yaw = state(4,:);

if nargin == 2
    params = varargin{1};
    [pos, yaw, dim] = convertToEgo(params, pos, yaw, dim);
end

meas = helperLidarModel(pos,dim,yaw);

end

function state = helperctcuboid(state, v, dT)

% Convention for state is [x;y;s;theta;omega;z;vz;L;W;H];

ctrectstate = state([1:5,8:9],:);
zstate = state(6:7,:);
h = state(10,:);

if isscalar(v) || nargin == 2
    dT = v;
    vNoise = zeros(3,size(state,2));
else
    vNoise = v;
end

ctrectstatedt = ctrect(ctrectstate,vNoise(1:2,:),dT);
zstatedt = constvel(zstate,vNoise(3,:),dT);
hdt = h;

state([1:5,8:9],:) = ctrectstatedt;
state(6:7,:) = zstatedt;
state(10,:) = hdt;

end

function state = helpercvcuboid(state, v, dT)
% Convention for state is [x;y;s;theta;z;vz;L;W;H]

if isscalar(v) || nargin == 2
    dT = v;
    vNoise = zeros(3,size(state,2));
else
    vNoise = v;
end

% Its same as ctstate, except that omega is noise now and alpha is 0.
omega = vNoise(2,:);
valpha = zeros(1,size(state,2));

statect = [state(1:4,:);omega;state(5:end)];
vct = [vNoise(1,:);valpha;vNoise(3,:)];

statectdt = helperctcuboid(statect, vct, dT);

state(1:4,:) = statectdt(1:4,:);
state(5:end,:) = statectdt(6:end,:);

end

function meas = helpercvcuboidmeas(state, varargin)

% Convention for state is [x;y;s;theta;z;vz;L;W;H];
pos = state([1 2 5],:);
dim = state([7 8 9],:);
yaw = state(4,:);

if nargin == 2
    params = varargin{1};
    [pos, yaw, dim] = convertToEgo(params, pos, yaw, dim);
end

meas = helperLidarModel(pos,dim,yaw);

end

function [pos, yaw, dim] = convertToEgo(egoParams, pos, yaw, dim)

egoPos = egoParams.OriginPosition;
egoOrientation = egoParams.Orientation;
ypr = eulerd(quaternion(egoOrientation,'rotmat','frame'),'ZYX','frame');
pos = egoOrientation*(bsxfun(@minus,pos,egoPos(:)));

if nargin > 2
    egoYaw = ypr(1);
    yaw = yaw - egoYaw;
end

end

function st = measParamStruct

st = struct('Frame',drivingCoordinateFrameType(1),...
    'OriginPosition',zeros(3,1),...
    'OriginVelocity',zeros(3,1),...
    'Orientation',eye(3));
end

    