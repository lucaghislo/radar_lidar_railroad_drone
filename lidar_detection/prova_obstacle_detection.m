% Read a scan of lidar data
ptCloud = sensors(1).PointClouds{1, 1};


%% Setup Streaming Point Cloud Display
xlimits = [-25 45]; % meters
ylimits = [-25 45];
zlimits = [-20 20];

% Create a pcplayer
lidarViewer = pcplayer(xlimits, ylimits, zlimits);
    
% Customize player axes labels
xlabel(lidarViewer.Axes, 'X (m)')
ylabel(lidarViewer.Axes, 'Y (m)')
zlabel(lidarViewer.Axes, 'Z (m)')

% Display the raw lidar scan
view(lidarViewer, ptCloud)

% Define labels to use for segmented points
colorLabels = [...
    0      0.4470 0.7410; ... % Unlabeled points, specified as [R,G,B]
    0.4660 0.6740 0.1880; ... % Ground points
    0.9290 0.6940 0.1250; ... % Ego points
    0.6350 0.0780 0.1840];    % Obstacle points

% Define indices for each label
colors.Unlabeled = 1;
colors.Ground    = 2;
colors.Ego       = 3;
colors.Obstacle  = 4;

% Set the colormap
colormap(lidarViewer.Axes, colorLabels)

mountLocation = [...
    vehicleDims.Length/2 - vehicleDims.RearOverhang, ... % x
    0, ...                                               % y
    vehicleDims.Height];                                 % z

% Segment the ego vehicle using the helper function
points = struct();
points.EgoPoints = lidar_segmentation_function(ptCloud, vehicleDims, mountLocation);

%Visualize the point cloud with segmented ego vehicle. Use the
closePlayer = false;
helperUpdateView(lidarViewer, ptCloud, points, colors, closePlayer);


%% Segment Ground Plane and Nearby Obstacles
elevationDelta = 10;
points.GroundPoints = segmentGroundFromLidarData(ptCloud, 'ElevationAngleDelta', elevationDelta);

% Visualize the segmented ground plane.
helperUpdateView(lidarViewer, ptCloud, points, colors, closePlayer);
nonEgoGroundPoints = ~points.EgoPoints & ~points.GroundPoints;
ptCloudSegmented = select(ptCloud, nonEgoGroundPoints, 'OutputSize', 'full');

sensorLocation  = [0, 0, 0]; % Sensor is at the center of the coordinate system
radius          = 40; % meters

points.ObstaclePoints = findNeighborsInRadius(ptCloudSegmented, ...
    sensorLocation, radius);

% Visualize the segmented obstacles
helperUpdateView(lidarViewer, ptCloud, points, colors, closePlayer);


%% Process Lidar Sequence
i = 1;
while i<126
    % Grab the next lidar scan
    ptCloud = sensors(i).PointClouds{1, 1};
    
    % Segment points belonging to the ego vehicle
    points.EgoPoints = lidar_segmentation_function(ptCloud, vehicleDims, mountLocation);
    
    % Segment points belonging to the ground plane
    points.GroundPoints = segmentGroundFromLidarData(ptCloud, 'ElevationAngleDelta', elevationDelta);
    
    % Remove points belonging to the ego vehicle and ground plane
    nonEgoGroundPoints = ~points.EgoPoints & ~points.GroundPoints;
    ptCloudSegmented = select(ptCloud, nonEgoGroundPoints, 'OutputSize', 'full');
    
    % Segment obstacles
    points.ObstaclePoints = findNeighborsInRadius(ptCloudSegmented, sensorLocation, radius);
    
    closePlayer = 0;

    % Update lidar display
    isPlayerOpen = helperUpdateView(lidarViewer, ptCloud, points, colors, closePlayer);

    i = i+1 %#ok<NOPTS>     
end
snapnow


%% |helperUpdateView| updates the streaming point cloud display with the
function isPlayerOpen = helperUpdateView(lidarViewer, ptCloud, points, colors, closePlayer)
if closePlayer
    hide(lidarViewer);
    isPlayerOpen = false;
    return;
end
    
scanSize = size(ptCloud.Location);
scanSize = scanSize(1:2);

% Initialize colormap
colormapValues = ones(scanSize, 'like', ptCloud.Location) * colors.Unlabeled;

if isfield(points, 'GroundPoints')
    colormapValues(points.GroundPoints) = colors.Ground;
end

if isfield(points, 'EgoPoints')
    colormapValues(points.EgoPoints) = colors.Ego;
end

if isfield(points, 'ObstaclePoints')
    colormapValues(points.ObstaclePoints) = colors.Obstacle;
end

% Update view
view(lidarViewer, ptCloud.Location, colormapValues)

% Check if player is open
isPlayerOpen = isOpen(lidarViewer);

end