clear
close all
clc

%% Get latest log file

% Get a list of all .log files in the current directory
logFiles = dir('*.log');

% Check if any log files exist
if isempty(logFiles)
    error('No .log files found in the current directory.');
end

% Sort the files by date (latest first)
[~, idx] = sort([logFiles.datenum], 'descend');

% Get the name of the most recent file
latestLogFile = logFiles(idx(1)).name;


%% Plot

% See logger.h for format of the file
csv = readtable(latestLogFile);
dd = table2struct(csv, 'ToScalar', true);


figure();


%%% Velocity %%%
subplot(3,1,1); hold on; grid on; legend;
plot(dd.time, dd.vel_ref, 'DisplayName', 'Velocity Reference');
plot(dd.time, dd.velocity, 'DisplayName', 'Velocity');
ylabel('Speed  [m/s]');

%%% Heading %%%
subplot(3,1,2); hold on; grid on; legend;
plot(dd.time, dd.heading_ref, 'DisplayName', 'Heading Reference');
plot(dd.time, dd.heading, 'DisplayName', 'Heading');
ylabel('Angle  [rad]');

%%% Turnrate %%%
subplot(3,1,3); hold on; grid on; legend;
plot(dd.time, dd.turnrate_ref, 'DisplayName', 'Turnrate Reference');
plot(dd.time, dd.turnrate, 'DisplayName', 'Turnrate');
plot(dd.time, deg2rad(dd.gyroZ), '--', 'DisplayName', 'Gyro');
ylabel('Angular velocity  [rad/s]');

xlabel('Time  [ms]');


%% Plot pose

figure();
hold on;

% Plot markers on start and end
plot(dd.x(1), dd.y(1), 'o', 'Color','blue', 'MarkerSize',10, 'LineWidth',2);
plot(dd.x(end), dd.y(end), 'x', 'Color','red', 'MarkerSize',10, 'LineWidth',2);

% Plot the robot's trajectory
plot(dd.x, dd.y, 'b-', 'LineWidth', 2, 'Color', "#0072BD");

%quiver(dd.x, dd.y, cos(dd.heading), sin(dd.heading), 0.2, 'r'); % Add heading vectors

% Add labels and title
xlabel('X Position (meters)');
ylabel('Y Position (meters)');
title('Robot Trajectory');
legend('Start', 'End');
axis equal;







