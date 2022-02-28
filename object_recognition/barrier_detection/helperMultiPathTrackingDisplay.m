classdef helperMultiPathTrackingDisplay < matlab.System
    % This is a helper class for multipath radar tracking example to
    % display tracks, detections and ground truth. It may be removed or
    % modified in a future release.
    
    % Copyright 2020 The MathWorks, Inc.
    
    % Public properties
    properties
        Figure
        BirdsEyePlots
        % ActorID to follow in the panel
        FollowActorID = [];
    end
    
    properties
        Record = true;
        pFrames = cell(0,1);
    end
    
    properties
        FigureSnapshots
        SnapshotTimes = [3 4.3];
        ColorOrder = standardColorOrder();
    end
    
    methods
        function obj = helperMultiPathTrackingDisplay(varargin)
            setProperties(obj,nargin,varargin{:});
            % Make a figure
            hFigure = figure('Position', [0, 0, 1200, 640], 'Name', 'Multipath Radar Object Tracking Example');
            set(hFigure,'Visible','off')
            movegui(hFigure, [0 -1]); % Moves the figure to the left and a little down from the top
            obj.Figure = hFigure;
        end
        
        
        function writeAnimation(obj, fName, delay)
            if nargin == 2
                delay = 0;
            end
            downSample = 2;
            if obj.Record
                frames = obj.pFrames;
                imSize = size(frames{1}.cdata);
                im = zeros(imSize(1),imSize(2),1,floor(numel(frames)/downSample),'uint8');
                map = [];
                count = 1;
                for i = 1:downSample:numel(frames)
                    if isempty(map)
                        [im(:,:,1,count),map] = rgb2ind(frames{i}.cdata,256,'nodither');
                    else
                        im(:,:,1,count) = rgb2ind(frames{i}.cdata,map,'nodither');
                    end
                    count = count + 1;
                end
                imwrite(im,map,[fName,'.gif'],'DelayTime',delay,'LoopCount',inf);
            end
        end
        
        function writeMovie(obj, fName, frameRate)
            vidWriter = VideoWriter([fName,'.avi']);
            vidWriter.FrameRate = frameRate;
            vidWriter.open();
            for i = 1:numel(obj.pFrames)
                vidWriter.writeVideo(obj.pFrames{i});
            end
            vidWriter.close();
        end
    end
    
     methods
        function snapnow(obj)
            if isPublishing(obj)
                panels = findall(obj.Figure,'Type','uipanel');
                f = copy(panels);
                obj.FigureSnapshots{end+1} = f;
            end
        end
        
        function fOut = showSnaps(obj, panelIdx, idx)
            fOut = [];
            if isPublishing(obj)
                if nargin == 1
                    idx = 1:numel(obj.FigureSnapshots);
                    panelIdx = 1:numel(obj.FigureSnapshots{1});
                end

                fOut = gobjects(numel(idx),1);

                for i = 1:numel(idx)
                    f = figure('Units','normalized','Position',[0 0 0.8 0.8]);
                    
                    for k = 1:numel(panelIdx)
                        copyobj(obj.FigureSnapshots{idx(i)}(panelIdx(k)),f);
                    end
                    panel = findall(f,'Type','uipanel');
                    if numel(panelIdx) == 1
                        panel.Position = [0 0 1 1];
                    end
                    ax = findall(panel,'Type','Axes');
                    for k = 1:numel(ax)
                        ax(k).XTickLabelMode = 'auto';
                        ax(k).YTickLabelMode = 'auto';
                    end
                    uistack(panel,'top');
                    fOut(i) = f;
                end
            end
        end
        
        function takeSnapshots(obj, scenario)
            time = scenario.SimulationTime;
            if any(abs(time - obj.SnapshotTimes) < 1e-3)
                snapnow(obj);
            end
        end
        
        function tf = isPublishing(~)
            tf = false;
            try
                s = numel(dbstack);
                tf = s > 5;
            catch
            end
        end
     end
        
    methods (Access = protected)
        function setupImpl(obj, egoVehicle, sensors)
            obj.BirdsEyePlots = createDemoDisplay(obj,egoVehicle,sensors);
        end
        
        function stepImpl(obj, egoVehicle, ~,  detections,tracks, varargin)
            bep = obj.BirdsEyePlots;
            
            for i = 1:numel(bep)
                setupCallback(bep{i},detections,targetPoses(egoVehicle),actorProfiles(egoVehicle.Scenario));
            end
            
            tracksEgo = helperConvertToEgoCoordinates(egoVehicle, tracks);
            
            % Update plots
            helperUpdateDisplayExtended(bep,egoVehicle,detections, tracksEgo, varargin{:});
            
            % Follow actor ID provided in actor ID
            if ~isempty(obj.FollowActorID)
                scene = egoVehicle.Scenario;
                pos = targetOutlines(egoVehicle);
                actorPos = pos(ismember([scene.Actors.ActorID],obj.FollowActorID),:);
                minX = min(actorPos(:,1));
                maxX = max(actorPos(:,1));
                minY = min(actorPos(:,2));
                maxY = max(actorPos(:,2));
                bep{1}.XLimits = [minX-25 maxX+25];
                bep{1}.YLimits = [minY-15 maxY+15];
            end
            
            if obj.Record
                obj.pFrames{end+1} = getframe(obj.Figure);
            end
            
            takeSnapshots(obj, egoVehicle.Scenario);
        end
        
        
        function BEPS = createDemoDisplay(obj, egoCar, sensors)
            hFigure = obj.Figure;
            
            % Add a car plot that follows the ego vehicle from behind
            hCarViewPanel = uipanel(hFigure, 'Position', [0 0 0.4 1], 'Title', 'Chase Camera View');
            hCarPlot = axes(hCarViewPanel);
            chasePlot(egoCar, 'Parent', hCarPlot);
            
            % Create panels with bird's-eye plots
            CenterBEP = createBEPPanel(obj, hFigure, [0.4 0 0.6 1], 60, sensors, 'Top View', true);
            
            BEPS = {CenterBEP};
            if ~obj.isPublishing()
                set(hFigure,'Visible','on');
            else
                % Delete legend is publishing
                legend(CenterBEP.Parent,'off');
            end            
        end
    end
end


function BEP = createBEPPanel(obj, hFigure, position, frontBackLim, sensors, title, isLegend)
    % Add a panel for a bird's-eye plot
    hBEVPanel = uipanel(hFigure, 'Position', position, 'Title', title);
    
    % Create bird's-eye plot for the ego car and sensor coverage
    hBEVPlot = axes(hBEVPanel, 'Tag', 'birdsEyePlotAxes');
    BEP = birdsEyePlot('Parent', hBEVPlot, 'Xlimits', [-frontBackLim frontBackLim], 'Ylimits', [-35 35]);
    
    axis(BEP.Parent, 'equal');
    xlim(BEP.Parent, [-frontBackLim frontBackLim]);
    ylim(BEP.Parent, [-frontBackLim frontBackLim]*0.667);
    legend(hBEVPlot, 'off')
    
    clrs = obj.ColorOrder;
    
    % Create a radar detection plotter
    detectionPlotter(BEP, 'DisplayName','Targets', 'Marker','o','MarkerEdgeColor',clrs(8,:),'MarkerFaceColor',clrs(7,:),'MarkerSize',6);
    
    % Create a ghost radar detection plotter
    detectionPlotter(BEP, 'DisplayName','Ghost (S)', 'Marker','o','MarkerEdgeColor',clrs(8,:),'MarkerFaceColor',clrs(5,:),'MarkerSize',6);
    
    % Create a dynamic ghost radar detection plotter
    detectionPlotter(BEP, 'DisplayName','Ghost (D)', 'Marker','o','MarkerEdgeColor',clrs(8,:),'MarkerFaceColor',clrs(1,:),'MarkerSize',6);
    
    % Create a static radar detection plotter
    detectionPlotter(BEP, 'DisplayName','Static', 'Marker','.','MarkerEdgeColor',clrs(8,:),'MarkerFaceColor',clrs(8,:),'MarkerSize',10);
    
    % Create a lane marking plotter
    laneMarkingPlotter(BEP, 'DisplayName','lane');
    
    % Create a track plotter
    trackPlotter(BEP, 'DisplayName','track', 'HistoryDepth',10);
    
    % Add an outline plotter for ground truth
    outlinePlotter(BEP, 'Tag', 'Ground truth');
    
    % Add a plotter for extent for plotting track outlines of Elliptical
    % targets
    trackPlotter(BEP, 'Tag', 'Track Extent');
    s = findall(BEP.Parent,'Tag','bepTracksCovariances');
    s.LineStyle = ':';
    s.LineWidth = 2;
    
    % Add a line plotter for plotting reflectors
    hold(BEP.Parent,'on');
    plot(BEP.Parent, nan,nan,'-','LineWidth',3,'DisplayName','Reflectors', 'Tag', 'Reflectors','Color',clrs(8,:));
    
    if ~isempty(sensors)
        % Plot the coverage areas for radars
        for i = 1:numel(sensors)
            if isa(sensors{i},'radarDataGenerator')
                cap = coverageAreaPlotter(BEP,'FaceColor','red','EdgeColor','red');
                plotCoverageArea(cap, sensors{i}.MountingLocation(1:2),...
                    sensors{i}.RangeLimits(2), sensors{i}.MountingAngles(1), sensors{i}.FieldOfView(1));
            else
                cap = coverageAreaPlotter(BEP,'FaceColor','blue','EdgeColor','blue');
                plotCoverageArea(cap, sensors{i}.SensorLocation(1:2),...
                    sensors{i}.MaxRange, sensors{i}.Yaw, 45);
            end
        end
    end
    
    if ~isLegend
        
    else
        l = legend('Orientation','horizontal','NumColumns',1);
        l.Location = 'northeastoutside';
    end
end

function helperUpdateDisplayExtended(BEPS, egoCar, detections, confirmedTracks, varargin)
%%% 
% helperUpdateDisplayExtended  Helper to update the display with extended tracks
% 
% This function updates the bird's-eye plot with road boundaries,
% detections, and extended tracks.
    for b = 1:numel(BEPS)
        BEP = BEPS{b};
        helperUpdateDisplayNonTracks(BEP, egoCar, detections, varargin{:});
        trackIDs = [confirmedTracks.TrackID];
        if isfield(confirmedTracks,'SourceIndex') % PHD
            if isfield(confirmedTracks,'Extent')
                [trackPos, trackExtent, trackVel] = trackOutlinesGGIW(confirmedTracks);
                plotTrack(findPlotter(BEP, 'Tag','Track Extent'), trackPos, trackExtent, trackVel, string(trackIDs));
            else
                [tracksPos, yaw, length, width] = tracksOutlines(confirmedTracks);
                plotTrack(findPlotter(BEP,'DisplayName','track'), tracksPos, string(trackIDs));
                plotOutline(findPlotter(BEP,'Tag','Track Outlines'), tracksPos, yaw, length, width,...
                    'Color', ones(numel(confirmedTracks), 3) * 0.1 ...
                    );
            end
        else % multiObjectTracker
            [trackPos, trackPosCov] = getTrackPositions(confirmedTracks,[1 0 0 0 0;0 0 1 0 0]);
            plotTrack(findPlotter(BEP,'DisplayName','track'), trackPos, trackPosCov, string(trackIDs));
        end
    end
end

function [pos,extent,vel] = trackOutlinesGGIW(tracks)
    pos = zeros(numel(tracks),2);
    vel = zeros(numel(tracks),2);
    extent = zeros(2,2,numel(tracks));
    for i = 1:numel(tracks)
        pos(i,:) = tracks(i).State([1 3]);
        vel(i,:) = tracks(i).State([2 4]);
        extent(:,:,i) = tracks(i).Extent/2 + tracks(i).Extent'/2;
    end
end

function [position, yaw, length, width] = tracksOutlines(tracks)
% tracksOutlines  Returns the track outlines for display

position = zeros(numel(tracks), 2);
yaw = zeros(numel(tracks), 1);
length = zeros(numel(tracks), 1);
width = zeros(numel(tracks), 1);

for i = 1:numel(tracks)
    position(i, :) = tracks(i).State(1:2)';
    yaw(i) = tracks(i).State(4);
    length(i, 1) = tracks(i).State(6);
    width(i, 1) = tracks(i).State(7);
end
end

function helperUpdateDisplayNonTracks(BEP,egoCar,detections, ghostDynamicDetections, ghostStaticDetections, staticDetections, reflectors)
%helperUpdateDisplayNonTracks  Helper to update display of all non-track plotters

% Update road boundaries and their display
[lmv, lmf] = laneMarkingVertices(egoCar);
plotLaneMarking(findPlotter(BEP,'DisplayName','lane'),lmv,lmf);

% update ground truth data
[position, yaw, length, width, originOffset, color] = targetOutlines(egoCar);
plotOutline(findPlotter(BEP,'Tag','Ground truth'), position, yaw, length, width, 'OriginOffset', originOffset, 'Color', color);

% Prepare and update detections display
pos = zeros(3,numel(detections));
vel = zeros(3,numel(detections));
sIdx = cellfun(@(x)x.SensorIndex,detections);

uqSIdx = unique(sIdx);
for i = 1:numel(uqSIdx)
    thisIdx = sIdx == uqSIdx(i);
    [pos(:,thisIdx), vel(:,thisIdx)] = calculatePositionInEgoFrame(detections(thisIdx));
end

plotDetection(findPlotter(BEP,'DisplayName','Targets'), pos(1:2,:)');

if nargin > 3
    pos = zeros(3,numel(ghostDynamicDetections));
    sIdx = cellfun(@(x)x.SensorIndex,ghostDynamicDetections);
    uqSIdx = unique(sIdx);
    for i = 1:numel(uqSIdx)
        thisIdx = sIdx == uqSIdx(i);
        [pos(:,thisIdx), vel(:,thisIdx)] = calculatePositionInEgoFrame(ghostDynamicDetections(thisIdx));
    end
    plotDetection(findPlotter(BEP,'DisplayName','Ghost (S)'), pos(1:2,:)');
end

if nargin > 4
    pos = zeros(3,numel(ghostStaticDetections));
    sIdx = cellfun(@(x)x.SensorIndex,ghostStaticDetections);
    uqSIdx = unique(sIdx);
    for i = 1:numel(uqSIdx)
        thisIdx = sIdx == uqSIdx(i);
        [pos(:,thisIdx), vel(:,thisIdx)] = calculatePositionInEgoFrame(ghostStaticDetections(thisIdx));
    end
    plotDetection(findPlotter(BEP,'DisplayName','Ghost (D)'), pos(1:2,:)');
end

if nargin > 5
    pos = zeros(3,numel(staticDetections));
    sIdx = cellfun(@(x)x.SensorIndex,staticDetections);
    uqSIdx = unique(sIdx);
    for i = 1:numel(uqSIdx)
        thisIdx = sIdx == uqSIdx(i);
        [pos(:,thisIdx), vel(:,thisIdx)] = calculatePositionInEgoFrame(staticDetections(thisIdx));
    end
    plotDetection(findPlotter(BEP,'DisplayName','Static'), pos(1:2,:)');
end

% Plot reflectors
if nargin > 6
   reflectors(:,3,:) = nan;
   x = reflectors(1,:);
   y = reflectors(2,:);
   plotter = findall(BEP.Parent,'Tag','Reflectors');
   plotter.XData = x;
   plotter.YData = y;
end
end

function tracks = helperConvertToEgoCoordinates(egoCar, tracks)
% helperConvertToEgoCoordinates
%
% This function converts the tracks from global coordinates to ego
% coordinates.

% Copyright 2018 The Mathworks, Inc.

x0 = egoCar.Position(1:2)';
v0 = egoCar.Velocity(1:2)';
phi = egoCar.Yaw;
rot = [cosd(phi), sind(phi); -sind(phi), cosd(phi)];
if isfield(tracks,'Extent')
    for i = 1:numel(tracks)
        trackPos = tracks(i).State([1 3]);
        trackVel = tracks(i).State([2 4]);
        yaw = atan2d(trackVel(2),trackVel(1));
        tracks(i).State([1 3]) = rot*(trackPos - x0);
        tracks(i).State([2 4]) = rot*(trackVel - v0);
        [~,extent] = eig(tracks(i).Extent);
        dims = sort(diag(extent));
        tracks(i).Extent = rot*tracks(i).Extent*rot';
    end
else
    for i = 1:numel(tracks)
        trackPos = tracks(i).State(1:2);
        yaw = tracks(i).State(4);
        trackVel = [tracks(i).State(3)*cosd(yaw);tracks(i).State(4)*sind(yaw)];
        tracks(i).State(1:2) = rot * (trackPos - x0);
        tracks(i).State(3) = norm(trackVel - v0);
        tracks(i).State(4) = tracks(i).State(4) - phi;
    end
end

end

function [posEgo, velEgo] = calculatePositionInEgoFrame(detections)

% Calculate Cartesian positions for all detections in the "sensor"
% coordinate frame
allDets = [detections{:}];
meas = horzcat(allDets.Measurement);

if strcmpi(allDets(1).MeasurementParameters(1).Frame,'Spherical')
    az = meas(1,:);
    r = meas(2,:);
    el = zeros(1,numel(az));
    [x, y, z] = sph2cart(deg2rad(az),deg2rad(el),r);
    posSensor = [x;y;z];
    rr = meas(3,:);
    rVec = posSensor./sqrt(dot(posSensor,posSensor,1));
    velSensor = rr.*rVec;
else
    posSensor = meas;
    velSensor = zeros(3,size(meas,2));
end

% Transform parameters
sensorToEgo = detections{1}.MeasurementParameters(1);
R = sensorToEgo.Orientation;
T = sensorToEgo.OriginPosition;
if isfield(sensorToEgo,'OriginVelocity')
    Tdot = sensorToEgo.OriginVelocity;
else
    Tdot = zeros(3,1);
end

if isfield(sensorToEgo,'IsParentToChild') && sensorToEgo.IsParentToChild
    R = R';
end

% Position, velocity in ego frame
posEgo = T + R*posSensor;
velEgo = Tdot + R*velSensor; % Assume Rdot = 0;

end

%% Callback code
% This code is intended to assist debugging. By clicking on a detection,
% you can visualize the path resulting in the detection.
function setupCallback(bep, detections, tgts, profiles)

sIdx = cellfun(@(x)x.SensorIndex,detections);
uqSIdx = unique(sIdx);
posEgo = zeros(3,numel(detections));
for i = 1:numel(uqSIdx)
    thisIdx = sIdx == uqSIdx(i);
    posEgo(:,thisIdx) = calculatePositionInEgoFrame(detections(thisIdx));
end

hLines = findall(bep.Parent,'Type','Line');
hLine = hLines(startsWith(arrayfun(@(x)x.Tag,hLines,'UniformOutput',false),'bepDetectionPositions'));

for i = 1:numel(hLine)
    uistack(hLine,'top');
    hLine(i).ButtonDownFcn = @(src, event)plotMultipathReflections(src, event, bep, posEgo, detections, tgts, profiles);
end

end

function plotMultipathReflections(src, event, bep, posEgo, detections, targets, profiles)

ax = src.Parent;

plotter = findall(ax,'Tag','multiPathReflection');
pointPlotter = findall(ax,'Tag','pointInQuestion');

if isempty(plotter)
    plotter = plot(ax,nan,nan,'-.','LineWidth',2,'Tag','multiPathReflection');
    plotter.HitTest = 'off';
end

if isempty(pointPlotter)
    pointPlotter = plot(ax,nan,nan,'o','Color','green','Tag','pointInQuestion');
    pointPlotter.HitTest = 'off';
end

xyHit = event.IntersectionPoint;
xyDetections = posEgo';
dxy = sum((xyDetections - xyHit).^2,2);
[~,idx] = min(dxy);
dHit = detections{idx};

pointPlotter.XData = xyDetections(idx,1);
pointPlotter.YData = xyDetections(idx,2);

tgtIdx = dHit.ObjectAttributes{1}.TargetIndex;

% False alarm
if tgtIdx == -1
    return;
end

bounceTgtIdx = dHit.ObjectAttributes{1}.BounceTargetIndex;
bouncePathIdx = dHit.ObjectAttributes{1}.BouncePathIndex;

tgt = targets([targets.ActorID] == tgtIdx);
tgtProf = profiles([profiles.ActorID] == tgtIdx);
bounce = targets([targets.ActorID] == bounceTgtIdx);
bounceProf = profiles([profiles.ActorID] == bounceTgtIdx);

p1 = dHit.MeasurementParameters(1).OriginPosition;
p2 = tgt.Position(:);
if ~isempty(bounce)
    p3 = bounce.Position(:);
else
    p3 = tgt.Position(:);
end

switch bouncePathIdx
    case 0
        path = [p1 p2 p2 p1];
    case {1,2}
        path = [p1 p2 p3 p1];
    case 3
        path = [p1 p3 p2 p3 p1];
end

plotter.XData = path(1,:);
plotter.YData = path(2,:);

plotTarget(bep, [tgt;bounce], [tgtProf;bounceProf]);

end

function plotTarget(bep, tgts, profs)

op = findPlotter(bep,'Tag','MultipathOutlines');

if isempty(op)
    op = outlinePlotter(bep, 'Tag', 'MultipathOutlines','FaceAlpha',0);
end

position = zeros(numel(tgts),2);
yaw = zeros(numel(tgts),1);
length = zeros(numel(tgts),1);
width = zeros(numel(tgts),1);
originOffset = zeros(numel(tgts),2);
color = zeros(numel(tgts),3);
order = lines(numel(tgts));

for i = 1:numel(tgts)
    position(i,:) = tgts(i).Position(1:2);
    yaw(i) = tgts(i).Yaw;
    length(i) = profs(i).Length;
    width(i) = profs(i).Width;
    originOffset(i,:) = profs(i).OriginOffset(1:2);
    color(i,:) = order(i,:);
end

op.plotOutline(position, yaw, length, width, 'OriginOffset',originOffset, 'Color',color);

end

function colors = standardColorOrder()

colors = lines(7);
colors(8,:) = [0 0 0]; % Black
colorsExtra = [93 70 95;27 48 73;171 200 209;23 140 100]./255;
colors = [colors;colorsExtra];

end