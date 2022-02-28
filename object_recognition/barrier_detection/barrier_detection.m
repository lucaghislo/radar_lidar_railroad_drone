% Create the scenario
[scenario, egoVehicle] = createDrivingScenario;
sensors = createSensor(scenario);

% Load the recorded data
%load('trainScenario_straight_short.mat','detectionLog','configurationLog');

%%
% Configuration of the sensors from the recording to set up the tracker
[~, sensorConfigurations] = helperAssembleData(detectionLog{1},configurationLog{1});

% Configure the tracker to use the GGIW-PHD filter with constant turn-rate motion model
for i = 1:numel(sensorConfigurations)
    sensorConfigurations{i}.FilterInitializationFcn = @helperInitGGIWFilter;    
    sensorConfigurations{i}.SensorTransformFcn = @ctmeas;
end

% Create the tracker using trackerPHD with Name-value pairs
tracker = trackerPHD('SensorConfigurations', sensorConfigurations,...
    'PartitioningFcn',@(x)helperMultipathExamplePartitionFcn(x,2,5),...
    'AssignmentThreshold',450,...
    'ExtractionThreshold',0.8,...
    'ConfirmationThreshold',0.85,...
    'MergingThreshold',25,...
    'DeletionThreshold',1e-2,...
    'BirthRate',1e-2,...
    'HasSensorConfigurationsInput',true... 
    );

%%
% Create trackGOSPAMetric object to calculate GOSPA metric
gospaMetric = trackGOSPAMetric('Distance','custom', ...
    'DistanceFcn',@helperGOSPADistance, ...
    'CutoffDistance',35);

% Create display for visualization of results
display = helperMultiPathTrackingDisplay;

% Predicted track list for ghost filtering
predictedTracks = objectTrack.empty(0,1);

% Confusion matrix
confMat = zeros(5,5,numel(detectionLog));

% GOSPA metric
gospa = zeros(4,numel(detectionLog));

% Ground truth 
groundTruth = scenario.Actors(2:end);

for i = 1:numel(detectionLog)
    % Advance scene for visualization of ground truth
    advance(scenario);
    
    % Current time
    time = scenario.SimulationTime;
    
    % Detections and sensor configurations
    [detections, configurations] = helperAssembleData(detectionLog{i},configurationLog{i});
    
    % Predict confirmed tracks to current time for classifying ghosts
    if isLocked(tracker)
        predictedTracks = predictTracksToTime(tracker,'confirmed',time);
    end
    
    % Classify radar detections as targets, ghosts, or static environment
    [targets, ghostStatic, ghostDynamic, static, reflectors, classificationInfo] = helperClassifyRadarDetections(detections, egoVehicle, predictedTracks);
    
    % Pass detections from target and sensor configurations to the tracker
    confirmedTracks = tracker(targets, configurations, time);

    % Visualize the results
    display(egoVehicle, sensors, targets, confirmedTracks, ghostStatic, ghostDynamic, static, reflectors);
    
    % Calculate GOSPA metric
    [gospa(1, i),~,~,gospa(2,i),gospa(3,i),gospa(4,i)] = gospaMetric(confirmedTracks, groundTruth);
    
    % Get true classification information and generate confusion matrix
    trueClassificationInfo = helperTrueClassificationInfo(detections);
    confMat(:,:,i) = helperConfusionMatrix(trueClassificationInfo, classificationInfo);
end

%%
f = showSnaps(display,1:2,1);
if ~isempty(f)
    ax = findall(f,'Type','Axes','Tag','birdsEyePlotAxes');
    ax.XLim = [-10 30];
    ax.YLim = [-10 20];
end

%%
f = showSnaps(display,1:2,2);
if ~isempty(f)
    ax = findall(f,'Type','Axes','Tag','birdsEyePlotAxes');
    ax.XLim = [-10 30];
    ax.YLim = [-10 20];
end

%%
figure;
plot(gospa','LineWidth',2);
legend('GOSPA','Localization GOSPA','Missed-target GOSPA','False-tracks GOSPA');

%%
% Accumulate confusion matrix over all steps
confusionMatrix = sum(confMat,3);
numElements = sum(confusionMatrix,2);
numElemsTable = array2table(numElements,'VariableNames',{'Number of Detections'},'RowNames',{'Targets','Ghost (S)','Ghost (D)','Environment','Clutter'});
disp('True Information');disp(numElemsTable);
% Calculate percentages
percentMatrix = confusionMatrix./numElements*100;

percentMatrixTable = array2table(round(percentMatrix,2),'RowNames',{'Targets','Ghost (S)','Ghost (D)','Environment','Clutter'},...
    "VariableNames",{'Targets','Ghost (S)','Ghost (D)', 'Environment','Clutter'});

disp('True vs Predicted Confusion Matrix (%)');disp(percentMatrixTable);