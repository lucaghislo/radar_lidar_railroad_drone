function [allData, scenario, sensors] = trainScenario_short_allsensors()
%trainScenario_short_allsensors - Returns sensor detections
%    allData = trainScenario_short_allsensors returns sensor detections in a structure
%    with time for an internally defined scenario and sensor suite.
%
%    [allData, scenario, sensors] = trainScenario_short_allsensors optionally returns
%    the drivingScenario and detection generator objects.

% Generated by MATLAB(R) 9.11 (R2021b) and Automated Driving Toolbox 3.4 (R2021b).
% Generated on: 01-Mar-2022 14:31:56

% Create the drivingScenario object and ego car
[scenario, egoVehicle] = createDrivingScenario;

% Create all the sensors
[sensors, numSensors] = createSensors(scenario);

allData = struct('Time', {}, 'ActorPoses', {}, 'ObjectDetections', {}, 'LaneDetections', {}, 'PointClouds', {}, 'INSMeasurements', {});
running = true;
while running

    % Generate the target poses of all actors relative to the ego vehicle
    poses = targetPoses(egoVehicle);
    % Get the state of the ego vehicle
    actorState = state(egoVehicle);
    time  = scenario.SimulationTime;

    objectDetections = {};
    laneDetections   = [];
    ptClouds = {};
    insMeas = {};
    isValidTime = false(1, numSensors);
    isValidLaneTime = false(1, numSensors);
    isValidPointCloudTime = false(1, numSensors);
    isValidINSTime = false(1, numSensors);

    % Generate detections for each sensor
    for sensorIndex = 1:numSensors
        sensor = sensors{sensorIndex};
        % Generate the ego vehicle lane boundaries
        if isa(sensor, 'visionDetectionGenerator')
            maxLaneDetectionRange = min(500,sensor.MaxRange);
            lanes = laneBoundaries(egoVehicle, 'XDistance', linspace(-maxLaneDetectionRange, maxLaneDetectionRange, 101));
        end
        type = getDetectorOutput(sensor);
        if strcmp(type, 'Objects only')
            [objectDets, numObjects, isValidTime(sensorIndex)] = sensor(poses, time);
            objectDetections = [objectDetections; objectDets(1:numObjects)]; %#ok<AGROW>
        elseif strcmp(type, 'Lanes only')
            [laneDets, ~, isValidTime(sensorIndex)] = sensor(lanes, time);
            laneDetections   = [laneDetections laneDets]; %#ok<AGROW>
        elseif strcmp(type, 'Lanes and objects')
            [objectDets, numObjects, isValidTime(sensorIndex), laneDets, ~, isValidLaneTime(sensorIndex)] = sensor(poses, lanes, time);
            objectDetections = [objectDetections; objectDets(1:numObjects)]; %#ok<AGROW>
            laneDetections   = [laneDetections laneDets]; %#ok<AGROW>
        elseif strcmp(type, 'Lanes with occlusion')
            [laneDets, ~, isValidLaneTime(sensorIndex)] = sensor(poses, lanes, time);
            laneDetections   = [laneDetections laneDets]; %#ok<AGROW>
        elseif strcmp(type, 'PointCloud')
            if sensor.HasRoadsInputPort
                rdmesh = roadMesh(egoVehicle,min(500,sensor.MaxRange));
                [ptCloud, isValidPointCloudTime(sensorIndex)] = sensor(poses, rdmesh, time);
            else
                [ptCloud, isValidPointCloudTime(sensorIndex)] = sensor(poses, time);
            end
            ptClouds = [ptClouds; ptCloud]; %#ok<AGROW>
        elseif strcmp(type, 'INSMeasurement')
            insMeasCurrent = sensor(actorState, time);
            insMeas = [insMeas; insMeasCurrent]; %#ok<AGROW>
            isValidINSTime(sensorIndex) = true;
        end
    end

    % Aggregate all detections into a structure for later use
    if any(isValidTime) || any(isValidLaneTime) || any(isValidPointCloudTime) || any(isValidINSTime)
        allData(end + 1) = struct( ...
            'Time',       scenario.SimulationTime, ...
            'ActorPoses', actorPoses(scenario), ...
            'ObjectDetections', {objectDetections}, ...
            'LaneDetections', {laneDetections}, ...
            'PointClouds',   {ptClouds}, ... %#ok<AGROW>
            'INSMeasurements',   {insMeas}); %#ok<AGROW>
    end

    % Advance the scenario one time step and exit the loop if the scenario is complete
    running = advance(scenario);
end

% Restart the driving scenario to return the actors to their initial positions.
restart(scenario);

% Release all the sensor objects so they can be used again.
for sensorIndex = 1:numSensors
    release(sensors{sensorIndex});
end

%%%%%%%%%%%%%%%%%%%%
% Helper functions %
%%%%%%%%%%%%%%%%%%%%

% Units used in createSensors and createDrivingScenario
% Distance/Position - meters
% Speed             - meters/second
% Angles            - degrees
% RCS Pattern       - dBsm

function [sensors, numSensors] = createSensors(scenario)
% createSensors Returns all sensor objects to generate detections

% Assign into each sensor the physical and radar profiles for all actors
profiles = actorProfiles(scenario);
sensors{1} = lidarPointCloudGenerator('SensorIndex', 1, ...
    'SensorLocation', [0.05 0], ...
    'MaxRange', 50, ...
    'EgoVehicleActorID', 2, ...
    'ActorProfiles', profiles);
sensors{2} = drivingRadarDataGenerator('SensorIndex', 2, ...
    'MountingLocation', [1 0 0.2], ...
    'RangeLimits', [0 100], ...
    'TargetReportFormat', 'Detections', ...
    'FieldOfView', [30 5], ...
    'Profiles', profiles);
sensors{3} = drivingRadarDataGenerator('SensorIndex', 3, ...
    'MountingLocation', [-1 0 0.2], ...
    'MountingAngles', [-180 0 0], ...
    'RangeLimits', [0 100], ...
    'TargetReportFormat', 'Detections', ...
    'FieldOfView', [30 5], ...
    'Profiles', profiles);
sensors{4} = drivingRadarDataGenerator('SensorIndex', 4, ...
    'MountingLocation', [0.05 0 1], ...
    'MountingAngles', [90 0 0], ...
    'RangeLimits', [0 100], ...
    'TargetReportFormat', 'Detections', ...
    'FieldOfView', [160 5], ...
    'Profiles', profiles);
sensors{5} = drivingRadarDataGenerator('SensorIndex', 5, ...
    'MountingLocation', [0.05 0 0.2], ...
    'MountingAngles', [-90 0 0], ...
    'RangeLimits', [0 100], ...
    'TargetReportFormat', 'Detections', ...
    'FieldOfView', [160 5], ...
    'Profiles', profiles);
sensors{6} = visionDetectionGenerator('SensorIndex', 6, ...
    'SensorLocation', [1 0], ...
    'MaxRange', 100, ...
    'DetectorOutput', 'Objects only', ...
    'Intrinsics', cameraIntrinsics([1814.81018227767 1814.81018227767],[320 240],[480 640]), ...
    'ActorProfiles', profiles);
sensors{7} = insSensor('TimeInput', true, ...
    'MountingLocation', [0.05 0 0]);
numSensors = 7;

function [scenario, egoVehicle] = createDrivingScenario
% createDrivingScenario Returns the drivingScenario defined in the Designer

% Construct a drivingScenario object.
scenario = drivingScenario;

% Add all road segments
roadCenters = [150.7 -50.6 0;
    343.5 -50.9 0];
marking = laneMarking('Unmarked');
laneSpecification = lanespec(1, 'Width', 3.35, 'Marking', marking);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Rotaie');

roadCenters = [150.69 -47.21 -0.7;
    343.49 -47.51 -0.7];
marking = laneMarking('Unmarked');
lanetypes = laneType('Driving', 'Color', [0 0.6 0.2]);
laneSpecification = lanespec(1, 'Width', 3.35, 'Marking', marking, 'Type', lanetypes);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Vegetazione');

% Add the barriers
barrierCenters = [150.58 -48.894 -0.7;
    170.456 -48.925 -0.7;
    190.332 -48.956 -0.7;
    210.208 -48.987 -0.7;
    230.085 -49.018 -0.7;
    249.961 -49.049 -0.7;
    269.837 -49.08 -0.7;
    289.714 -49.11 -0.7;
    309.59 -49.141 -0.7;
    329.466 -49.172 -0.7;
    343.38 -49.194 -0.7];
barrier(scenario, barrierCenters, ...
    'ClassID', 5, ...
    'SegmentLength', 1, ...
    'Width', 0.3, ...
    'Height', 0.9, ...
    'Mesh', driving.scenario.jerseyBarrierMesh, 'PlotColor', [0.65 0.65 0.65], 'Name', 'Jersey Barrier');

% Add the actors
train = vehicle(scenario, ...
    'ClassID', 7, ...
    'Length', 20, ...
    'Width', 2.8, ...
    'Height', 4, ...
    'Position', [324.25 -50.89 0.01], ...
    'Name', 'Train');
waypoints = [324.25 -50.89 0.01;
    158.57 -50.57 0.01];
speed = [30;30];
trajectory(train, waypoints, speed);

% Add the ego vehicle
egoVehicle = vehicle(scenario, ...
    'ClassID', 8, ...
    'Length', 2, ...
    'Width', 0.6, ...
    'Height', 1, ...
    'Position', [168.709935535012 -47.2890788442276 -0.69], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Drone');
waypoints = [168.709935535012 -47.2890788442276 -0.69;
    341.3 -47.42 0.01];
speed = [1.38;1.38];
smoothTrajectory(egoVehicle, waypoints, speed);

actor(scenario, ...
    'ClassID', 4, ...
    'Length', 0.59, ...
    'Width', 0.82, ...
    'Height', 1.7, ...
    'Position', [174.42 -46.37 -0.69], ...
    'RCSPattern', [-8 -8;-8 -8], ...
    'Mesh', driving.scenario.pedestrianMesh, ...
    'Name', 'Pedestrian');

actor(scenario, ...
    'ClassID', 4, ...
    'Length', 0.59, ...
    'Width', 0.82, ...
    'Height', 1.7, ...
    'Position', [163.54 -47.68 -0.69], ...
    'RCSPattern', [-8 -8;-8 -8], ...
    'Mesh', driving.scenario.pedestrianMesh, ...
    'Name', 'Pedestrian1');

function output = getDetectorOutput(sensor)

if isa(sensor, 'visionDetectionGenerator')
    output = sensor.DetectorOutput;
elseif isa(sensor, 'lidarPointCloudGenerator')
    output = 'PointCloud';
elseif isa(sensor, 'insSensor')
    output = 'INSMeasurement';
else
    output = 'Objects only';
end

