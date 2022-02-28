addpath(fullfile(matlabroot,'toolbox','shared','tracking','fusionlib'));

rng default;
initialDist = 150; % m
initialSpeed = 50; % kph
brakeAccel = 3;    % m/s^2
finalDist = 1;     % m
[scenario, egoCar] = helperCreateSensorDemoScenario('FCW', initialDist, initialSpeed, brakeAccel, finalDist);

radarSensor = drivingRadarDataGenerator( ...
    'SensorIndex', 1, ...
    'TargetReportFormat', 'Detections', ...
    'UpdateRate', 10, ...
    'MountingLocation', [egoCar.Wheelbase+egoCar.FrontOverhang 0 0.2], ...
    'FieldOfView', [20 5], ...
    'RangeLimits', [0 150], ...
    'AzimuthResolution', 4, ...
    'RangeResolution', 2.5, ...
    'Profiles', actorProfiles(scenario))

%% Create display for FCW scenario
[bep, figScene] = helperCreateSensorDemoDisplay(scenario, egoCar, radarSensor);

metrics = struct;                 % Initialize struct to collect scenario metrics
while advance(scenario)           % Update vehicle positions
    gTruth = targetPoses(egoCar); % Get target positions in ego vehicle coordinates
    
    % Generate time-stamped radar detections
    time = scenario.SimulationTime;
    [dets, ~, isValidTime] = radarSensor(gTruth, time);
    
    if isValidTime
        % Update Bird's-Eye Plot with detections and road boundaries
        helperUpdateSensorDemoDisplay(bep, egoCar, radarSensor, dets);
        
        % Collect radar detections and ground truth for offline analysis
        metrics = helperCollectScenarioMetrics(metrics, gTruth, dets);
    end
    
    % Take a snapshot for the published example
    helperPublishSnapshot(figScene, time>=9.1);
end

%% Position Measurements
helperPlotSensorDemoDetections(metrics, 'position', 'reverse range', [-6 6]);

% Show rear overhang of target vehicle
tgtCar = scenario.Actors(2);
rearOverhang = tgtCar.RearOverhang;

subplot(1,2,1);
hold on; plot(-rearOverhang*[1 1], ylim, 'k'); hold off;
legend('Error', '2\sigma noise', 'Rear overhang');

%% Range from radar to target vehicle's rear side
radarRange = 30-(radarSensor.MountingLocation(1)+tgtCar.RearOverhang);

% Azimuth spanned by vehicle's rear side at 30 meters ground truth range
width = tgtCar.Width;
azSpan = rad2deg(width/radarRange)

%% Velocity Measurements
% Create passing scenario
leadDist = 40;  % m
speed = 50;     % kph
passSpeed = 70; % kph
[scenario, egoCar] = helperCreateSensorDemoScenario('Passing', leadDist, speed, passSpeed);

%% Configure radar for range-rate measurements
release(radarSensor);
radarSensor.HasRangeRate = true;
radarSensor.RangeRateResolution = 0.5; % m/s

% Use actor profiles for the passing car scenario
radarSensor.Profiles = actorProfiles(scenario);
snapTime = 6; % Simulation time to take snapshot for publishing
metrics = helperRunSensorDemoScenario(scenario, egoCar, radarSensor, snapTime);

helperPlotSensorDemoDetections(metrics, 'velocity', 'time', [-25 25]);
subplot(1,2,1);
legend('Lead car error', 'Lead car 2\sigma noise', ...
    'Pass car error', 'Pass car 2\sigma noise', 'Location', 'northwest');

%% FCW Driving Scenario with a Pedestrian and a Vehicle
% Create FCW test scenario
initialDist = 150;  % m
finalDist = 1;      % m
initialSpeed = 50;  % kph
brakeAccel = 3;     % m/s^2
withPedestrian = true;
[scenario, egoCar] = helperCreateSensorDemoScenario('FCW', initialDist, initialSpeed, brakeAccel, finalDist, withPedestrian);

% Configure radar's long-range detection performance
release(radarSensor);
radarSensor.ReferenceRange = 100; % m
radarSensor.ReferenceRCS = 0;     % dBsm
radarSensor.DetectionProbability = 0.9;

% Use actor profiles for the passing car scenario
radarSensor.Profiles = actorProfiles(scenario);
snapTime = 8; % Simulation time to take snapshot for publishing
metrics = helperRunSensorDemoScenario(scenario, egoCar, radarSensor, snapTime);
helperPlotSensorDemoDetections(metrics, 'snr', 'range', [0 160]);
legend('Vehicle', 'Pedestrian');

% Configure radar for a mid-range detection requirement
release(radarSensor);
radarSensor.ReferenceRange = 50; % m
radarSensor.ReferenceRCS = 0;    % dBsm
radarSensor.DetectionProbability = 0.9;

%% Increase radar's field of view in azimuth and elevation to 90 and 10 degrees respectively
radarSensor.FieldOfView = [90 10];

% Increase radar's azimuth resolution
radarSensor.AzimuthResolution = 10;

% Run simulation and collect detections and ground truth for offline analysis
metrics = helperRunSensorDemoScenario(scenario, egoCar, radarSensor);

% Plot SNR for vehicle and pedestrian detections
helperPlotSensorDemoDetections(metrics, 'snr', 'range', [0 160]);
legend('Vehicle', 'Pedestrian');

%% Detection of Closely Spaced Targets
duration = 8;         % s
speedEgo = 50;        % kph
speedMotorcycles = 60; % kph
distMotorcycles = 25;  % m
[scenario, egoCar] = helperCreateSensorDemoScenario('Side-by-Side', duration, speedEgo, speedMotorcycles, distMotorcycles);

% Create forward-facing long-range automotive radar sensor mounted on ego vehicle's front bumper
radarSensor = drivingRadarDataGenerator(...
    'SensorIndex', 1, ...
    'TargetReportFormat', 'Detections', ...
    'MountingLocation', [egoCar.Wheelbase+egoCar.FrontOverhang 0 0.2], ...
    'Profiles', actorProfiles(scenario));

% Run simulation and collect detections and ground truth for offline analysis
snapTime = 5.6; % Simulation time to take snapshot for publishing
metrics = helperRunSensorDemoScenario(scenario, egoCar, radarSensor, snapTime);

helperPlotSensorDemoDetections(metrics, 'position', 'range', [-3 3], true);
subplot(1,2,2);
legend('Left error', 'Right error', 'Merged error');