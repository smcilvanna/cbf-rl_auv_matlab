






























%% SWEEP 1 - SINGLE OBSTACLE - Across Multiple .mat files
%% Read Mat Files to 3 info arrays


% Specify the data directory
directory = 'D:\matlab_data\2410_cbf_rl_auv\data_sweep1\'; % Replace with your directory path

% Get a list of all .mat files in the directory
mat_files = dir(fullfile(directory, '*.mat'));

% Sort the files by date
[~, idx] = sort([mat_files.datenum]);

% Extract the sorted file names
sorted_file_names = {mat_files(idx).name}';

% loop around sorted filenames
all_idx = [];
all_dst = [];
all_sep = [];
start_idx = 1;

for o = 1:size(sorted_file_names,1)
    load(append('./data/',sorted_file_names{o}));
    [idxs, distance, min_cen_sep] = processMatFile(start_idx,allData);
    all_idx = [all_idx ; idxs];
    all_dst = [all_dst ; distance];
    all_sep = [all_sep ; min_cen_sep];
    start_idx = idxs(end) + 1;
end







%%

k_values = [0.1, 0.5:0.5:150];
[X,Y] = ndgrid(k_values,k_values);

testValues = [X(:), Y(:)];
clearvars k_values X Y

%%

T                           = table( all_idx, testValues(:,1), testValues(:,2), all_dst, all_sep );
T.Properties.VariableNames  =       {'idx'  , 'k1'           , 'k2'           , 'dist' , 'csep'   };

T.ssep = T.csep - 1.05;


%%
% Calculate minimum path distance

% Define the length, pitch, and yaw
length = 1.05;
pitch = -45;
yaw = -45;
start = [0 0 0];
mid = [5 5 5] + calculateEndPoint(length, pitch, yaw);
last = [10 10 10];
L1 = norm(mid);
L2 = norm(last-mid);
opt_dist = L1 + L2;
% disp(opt_dist)
clearvars length pitch yaw start mid last L1 L2


% apply distance reward
rewardTable = T;
rewardTable.reward = zeros(height(T), 1);
% for row = 1:size(table,1)
%     if table(row).ssep < 0
%         rw = -1;
%     else
%         rw = opt_dist / table(row).ssep;
%     end
%     table(row).reward = rw;
% end
for row = 1:size(rewardTable, 1)
    if rewardTable.ssep(row) < 0
        rw = -1;
    else
        rw = opt_dist / rewardTable.dist(row);
        rw = rw^9; % scaling the reward for faster roll off as distance increases
    end
    rewardTable.reward(row) = rw;
end



%% PLOT

close all;
% Extract columns x, y, and z


range = 10;

plotTable = rewardTable(rewardTable.k1 <= range & rewardTable.k2 <= range, :);

x = plotTable.k1;
y = plotTable.k2;
z = plotTable.reward;

% Create a grid for x and y
[X, Y] = meshgrid(unique(x), unique(y));

% Use griddata to interpolate z values on the grid
Z = griddata(x, y, z, X, Y);

% Create the 3D surface plot
figure;
surf(X, Y, Z);

% Improve coloring and appearance
colormap(jet); % Change the colormap to 'jet' for better color variation
shading interp; % Interpolate shading to smooth out the surface
colorbar; % Add a colorbar to indicate the color scale

% Add labels and title
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('3D Surface Plot');

% Display the plot
grid on;

%% LOCAL FUNCTIONS


% Local Function for sweep1 mat file processing
function [idxs, distance, min_cen_sep] = processMatFile(idxStart,allData)
    
    cnt = idxStart;
    n = size(allData,1);
    idxs = zeros(n,1);
    distance = zeros(n,1);
    min_cen_sep = zeros(n,1);

    for i = 1:n

        if strcmp(allData(i).SimulationMetadata.ExecutionInfo.StopEvent,'ReachedStopTime')
            state = squeeze(allData(i).state.Data)';
            total_distance = 0;
            min_csep = 10;
            for j = 2:size(state,1)
                dst = norm( state(j,1:3) - state(j-1,1:3));
                total_distance = total_distance + dst;
                csep = norm(state(j,1:3) - [5,5,5] );
                min_csep = min(min_csep,csep);
            end
        else
            total_distance = -100;
            min_csep = -100;
        end
        idxs(i,1) = cnt;
        cnt = cnt + 1;
        distance(i,1) = total_distance;
        min_cen_sep(i,1) = min_csep;
    end
end

function end_point = calculateEndPoint(length, pitch, yaw)
    % Convert pitch and yaw to radians
    pitch_rad = deg2rad(pitch);
    yaw_rad = deg2rad(yaw);

    % Calculate the end point using spherical to Cartesian conversion
    x = length * cos(pitch_rad) * cos(yaw_rad);
    y = length * cos(pitch_rad) * sin(yaw_rad);
    z = length * sin(pitch_rad);

    % Define the end point
    end_point = [-x, y, z];

end
