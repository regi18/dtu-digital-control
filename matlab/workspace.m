clc
clear
close all

run("parameters.m");


%% Testing

%sim('final_robot_model.slx', 5);

%return


%% Do the path (1m straight -> 1m diameter loop -> 1.5m straight)

vel = 0.5;
h = "custom";

tot_dist = vel * 11;

path(:, 2) = path(:, 2) - 0.4;

out = set_and_go(vel, h, tot_dist);
plot_trajectory(out);


dist = out.pose.Data(:, 1);     % Distance
theta = out.pose.Data(:, 2);    % Heading
x = out.pose.Data(:, 3);        % X coordinate
y = out.pose.Data(:, 4);        % Y coordinate

%quiver(x(end), y(end), cos(theta(end)), sin(theta(end)), 0.8, 'r'); % Add heading vectors
yline(0, 'LineStyle','--');
yline(0.3, 'LineStyle','--'); 
d = 1;
px = 1.5-d/2;
py = 0.15-d/2;
h = rectangle('Position',[px py d d],'Curvature',[1,1], 'LineStyle','--');
daspect([1,1,1])





