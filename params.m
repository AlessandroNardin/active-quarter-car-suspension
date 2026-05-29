clear all;
close all;
clc;

%% suspension paramters
plant_param = struct();

plant_param.ms    = 350;
plant_param.mu    = 50;
plant_param.ks0   = 20000;
plant_param.alpha = 1e5;
plant_param.bs    = 1500;
plant_param.kt    = 180000;
plant_param.bt    = 150;

%% plant initial state
zs0 = 0;
zu0 = 0;
zsdot0 = 0;
zudot0 = 0;