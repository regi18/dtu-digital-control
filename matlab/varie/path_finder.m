clc
close all

image = imread('images/occupancy_map_mod.pgm');
imageNorm = double(image)/255;
imageOccupancy = imageNorm;
estMap = occupancyMap(imageOccupancy, 20);
inflate(estMap, 0.01)
show(estMap)



vMap = validatorOccupancyMap;
vMap.Map = estMap;
vMap.ValidationDistance = .1;
planner = plannerHybridAStar(vMap, 'MinTurningRadius', 1);

%%%%%%%%%
entrance = [0.5+(0.3/2) 0.55 0];
packagePickupLocation = [0.5-(0.3/2) 0.1 pi];
%%%%%%%%%


route = plan(planner, entrance, packagePickupLocation);
route = route.States;

% Get poses from the route.
rsConn = reedsSheppConnection('MinTurningRadius', planner.MinTurningRadius);
startPoses = route(1:end-1,:);
endPoses = route(2:end,:);

rsPathSegs = connect(rsConn, startPoses, endPoses);
poses = [];
for i = 1:numel(rsPathSegs)
    lengths = 0:0.1:rsPathSegs{i}.Length;
    [pose, ~] = interpolate(rsPathSegs{i}, lengths);
    poses = [poses; pose];
end

figure
show(planner)
title('Initial Route to Package')









