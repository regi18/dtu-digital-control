clc
clear
close all


%%

% path = [
%      0     4;
%      1     4;
%      2     4;
%      3     4;
%      4     4;
%      5     4;
%      6     4;
%      7     4;
%      8     4;
%      9     4;
%     10     4;
%     11     4;
%     12     4;
%     13     4;
%     14     4;
%     15     4;
% 
%     15     3;
%     16     2;
%     17     1;
%     18     1;
%     19     1;
%     20     1;
%     21     1;
%     22     1;
% 
%     23     2;
%     24     3;
%     24     4;
%     24     5;
%     24     6;
%     24     7;
%     24     8;
%     23     9;
% 
%     22    10;
%     20    10;
%     21    10;
%     19    10;
%     18    10;
%     17    10;
%     16     9;
%     15     8;
% 
%     15     7;
%     14     7;
%     13     7;
%     12     7;
%     11     7;
%     10     7;
%      9     7;
%      8     7;
%      7     7;
%      6     7;
%      5     7;
% ];

path = [
     5     4;
    15     4;
    15     3;
    %16     2;
    17     1;
    22     1;
    24     3;
    24     8;
    22    10;
    17    10;
    15     8;
    15     7;
     0     7;
];

path = path * 0.1;


% p = 0.9; % Smoothing parameter (0 to 1)
% path(:,1) = csaps(1:size(path,1), path(:,1), p, 1:size(path,1)); % Smooth x
% path(:,2) = csaps(1:size(path,1), path(:,2), p, 1:size(path,1)); % Smooth y

plot(path(:,1), path(:,2), 'k--d');


return


%% Simulation

robotInitialLocation = path(1,:);
robotGoal = path(end,:);
initialOrientation = 0;
robotCurrentPose = [robotInitialLocation initialOrientation]';
robot = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");


%%

controller = controllerPurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 10;
controller.MaxAngularVelocity = 2;
controller.LookaheadDistance = 0.3;

goalRadius = 0.1;
distanceToGoal = norm(robotInitialLocation - robotGoal);



%%

% Initialize the simulation loop
sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);

% Initialize the figure
figure;

% Determine vehicle frame size to most closely represent vehicle with plotTransforms
frameSize = robot.TrackWidth/0.8;

while( distanceToGoal > goalRadius )
    
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robotCurrentPose);
    
    % Get the robot's velocity using controller inputs
    vel = derivative(robot, robotCurrentPose, [v omega]);
    
    % Update the current pose
    robotCurrentPose = robotCurrentPose + vel*sampleTime; 
    
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    
    % Update the plot
    hold off
    
    % Plot path each instance so that it stays persistent while robot mesh
    % moves
    plot(path(:,1), path(:,2), "k--d")
    hold all
    
    % Plot the path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    light;
    xlim([0 25])
    ylim([0 10])

    waitfor(vizRate);
end








