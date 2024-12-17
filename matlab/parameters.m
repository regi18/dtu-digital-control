

%% References

vel_ref_start_time = 0.05;
vel_ref = 0.5;

%heading_ref_start_time = .10;
heading_ref = 0.9;


%% Parameters

M = 0.95;
wzi = (0.065)^2 * M;

Ts = 0.004;

wheel_distance = 0.147;

acceleration_limit = 3;     % m/s^2
turn_acc_limit = 5;         % rad/s

dist0 = 0;
theta0 = 0;
x0 = 0; 
y0 = 0;


%% Motor parameters

gear = 9.6;
wrad = 0.03;
transport_delay = 0.01;

% TODO  CHANGE THESE
R = 4.1; 
L = 4.1e-3; 
Kb = 0.0074;
Kt = Kb; 
D = 0.93e-6; 
J = 1.01e-6;


%% Controllers

% Load Kp, Ki
load('gains/motor_pi_gains');

% Load Kp_turnrate, Ki_turnrate, taud, alpha
load('gains/turnrate_pi_gains');

% Load Kp_heading, Ki_heading, taud_heading, alpha_heading
load('gains/heading_pi_gains');


K_lead_heading = 1;


%% Path planning controller

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

% Scale to real meters
path = path .* 0.1;

% headings = headingFromXY(path);
% figure
% hold on
% axis equal
% plot(path(:,1),path(:,2),"k-d")
% quiver(path(:,1),path(:,2),cos(headings),sin(headings),0.2)








