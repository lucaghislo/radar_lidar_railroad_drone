%pcwrite(sensors_curve(35).PointClouds{1, 1},'driving_scenario_PC_curve','PLYFormat','binary');

prefix = 'G:\My Drive\UniBG\CORSI\Laurea Magistrale\2021_2022 - Magistrale Secondo Anno\secondo_semestre\Tirocinio Formativo\tirocinio_PRSE\matlab_stuff\object_recognition\obstacle_detection\rolling_scenario_short\driving_scenario_PC_short_';

for i = 1:56
    post = num2str(i);
    pcwrite(sensors_short(i).PointClouds{1, 1}, [prefix, post],'PLYFormat','binary');
end