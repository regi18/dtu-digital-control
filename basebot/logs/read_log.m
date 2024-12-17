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

% Format of the file is:
%
% | time | ref_left | ref_right | left_motor_speed | right_motor_speed | state | PWM | left_current | right_current | left_motor_voltage | right_motor_voltage | 
%
dd = load(latestLogFile);

wheel_radius = 0.03;
gear = 9.6;
ratio = wheel_radius/gear;

time = dd(:,1);                
ref_left = dd(:,2) * ratio;     
ref_right = dd(:,3) * ratio;     
left_motor_speed = dd(:,4) * ratio;    
right_motor_speed = dd(:,5) * ratio;   
state = dd(:,6);               
PWM = dd(:,7);                 
left_current = dd(:,8);        
right_current = dd(:,9);       
left_motor_voltage = dd(:,10);  
right_motor_voltage = dd(:,11);

figure();
hold on

plot(time, ref_left, 'DisplayName', 'Reference (left)')
plot(time, ref_right, 'DisplayName', 'Reference (right)')
plot(time, right_motor_speed, 'DisplayName', 'Motor speed (right)')
plot(time, left_motor_speed, 'DisplayName', 'Motor speed (left)')

grid on
legend

ylabel('Speed  [m/s]');
xlabel('Time  [ms]');



