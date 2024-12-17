clc
close all

%%
map = binaryOccupancyMap(10, 25, 1);

occ = ones(25, 10);
val = 0;

% Start and end straight lines
occ(11:20, 7) = val;
occ(11:25, 4) = val;

% Circle
occ(10, [3,4,7,8]) = val;
occ(9, [2,9]) = val;
occ(8, [1,10]) = val;
occ(7, [1,10]) = val;
occ(6, [1,10]) = val;
occ(5, [1,10]) = val;
occ(4, [1,10]) = val;
occ(3, [1,10]) = val;
occ(2, [2,9]) = val;
occ(1, 3:8) = val;

setOccupancy(map, occ)


%% Save Image
mapMatrix = occupancyMatrix(map);
pgmData = uint8(255 * mapMatrix);  

% Upsample image
pgmData = imresize(pgmData, 2);

pgmFileName = 'occupancy_map.pgm';
imwrite(pgmData, pgmFileName);




