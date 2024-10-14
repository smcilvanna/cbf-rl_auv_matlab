k_values = [0.1:0.1:0.9 , 1.0:0.5:4.5, 5:1:15];
[X,Y] = ndgrid(k_values,k_values);
testValues = [X(:), Y(:)];
clearvars k_values X Y 
testObstacles = transpose(0.5:0.5:10);

rovRadius = 0.5;
obs_y = 0.0;
obs_z = 0.0;

% Open the Simulink model and enable fast restart
open_system('auv_nmpc_cbf.slx');
set_param('auv_nmpc_cbf', 'FastRestart', 'on');

for obs = 1:numel(testObstacles)
    allData = [];
    thisObsRadius = testObstacles(obs,1);

    obs_x = rovRadius + thisObsRadius + 5;
    obstacle = [obs_x , obs_y, obs_z, thisObsRadius];
    printTimeStamp
    fprintf(' >>>> Starting parameter sweep for obstacle radius %.2f m  | x-position %.3f\n',thisObsRadius,obs_x)
    for i = 1:size(testValues,1)
        cbfParms = testValues(i,:);
        data = sim("auv_nmpc_cbf.slx", 'CaptureErrors','on');
        allData = [allData ; data];
    end

    fname = "./data/sweep2_obs_" + sprintf('%02.1f', thisObsRadius) + ".mat";
    save(fname,"allData");
    pause(1);
    clearvars allData

end



%% LOCAL FUNCTIONS

function printTimeStamp()
% Get the current system time
currentTime = datetime('now');
% Round the seconds
roundedSeconds = round(second(currentTime));

% Format the time as a string
formattedTime = sprintf('%02d:%02d:%04d %02d:%02d:%02d', ...
    day(currentTime), month(currentTime), year(currentTime), ...
    hour(currentTime), minute(currentTime), roundedSeconds);

% Display the formatted time
disp(['Current system time: ', formattedTime]);

end