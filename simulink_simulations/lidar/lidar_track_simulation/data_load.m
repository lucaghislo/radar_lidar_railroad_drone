% Load the data if unavailable.
if ~exist('lidarData_1.mat','file')
    dataUrl = 'https://ssd.mathworks.com/supportfiles/lidar/data/TrackVehiclesUsingLidarExampleData.zip';
    datasetFolder = fullfile(pwd);
    unzip(dataUrl,datasetFolder);
end