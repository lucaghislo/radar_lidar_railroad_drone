%pcwrite(sensors_curve(35).PointClouds{1, 1},'driving_scenario_PC_curve','PLYFormat','binary');

prefix = 'G:\My Drive\UniBG\CORSI\Laurea Magistrale\2021_2022 - Magistrale Secondo Anno\secondo_semestre\Tirocinio Formativo\tirocinio_PRSE\matlab_stuff\object_recognition\obstacle_detection\curved_scenario\rolling_scenario_curve\driving_scenario_PC_curve_';

for i = 1:174
    post = num2str(i);
    pcwrite(sensors_curve(i).PointClouds{1, 1}, [prefix, post],'PLYFormat','binary');
end