function plot_trajectory(out, path)

x0 = evalin("base", "x0");
y0 = evalin("base", "y0");

dist = out.pose.Data(:, 1);     % Distance
theta = out.pose.Data(:, 2);    % Heading
x = out.pose.Data(:, 3);        % X coordinate
y = out.pose.Data(:, 4);        % Y coordinate

figure;
hold on;
grid on;

% Plot markers on start and end
plot(x0, y0, 'o', 'Color','blue', 'MarkerSize',10, 'LineWidth',2);
plot(x(end), y(end), 'x', 'Color','red', 'MarkerSize',10, 'LineWidth',2);

% Plot the robot's trajectory
plot(x, y, 'b-', 'LineWidth', 2, 'Color', "#0072BD");

%quiver(x, y, cos(theta), sin(theta), 0.2, 'r'); % Add heading vectors

% Add labels and title
xlabel('X Position (meters)');
ylabel('Y Position (meters)');
title('Robot Trajectory');
legend('Start', 'End');

axis equal;

if nargin == 2
    plot(path(:,1),path(:,2),"k-d", "DisplayName", "Path");
end

end