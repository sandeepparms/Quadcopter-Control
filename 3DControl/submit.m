close all;
clear;

% This is your controller
controlhandle = @controller;

% This is your trajectory generator
trajhandle = @traj_generator2;

evaluate(controlhandle, trajhandle);
