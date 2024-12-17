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

dd = load(latestLogFile);
time = dd(:,1);                

figure();
hold on
plotbrowser('on');

%%%%%
%vals = ["ref" "smoothedRef" "turnrate" "u"];
vals = ["headingRef" "heading"];
%%%%

for i = 1:length(vals)
    plot(time, dd(:, i+1), 'DisplayName', vals(i));
end

grid on
legend

ylabel('Speed  [m/s]');
xlabel('Time  [ms]');



