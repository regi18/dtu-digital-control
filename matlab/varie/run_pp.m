clc
clear
close all
addpath(genpath(".."));

parameters;

x0 = path(1,1); 
y0 = path(1,2);

vel_ref = 0.2;
lookahead_dist = 0.35;

Tfin = norm(path) / vel_ref;
out = sim("model_with_pursuit.slx", Tfin);

plot_trajectory(out, path);






