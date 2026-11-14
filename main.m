% main.m : reproduit toutes les figures du diaporama
clear; clc; close all;
racine = fileparts(mfilename('fullpath'));
cd(racine);
addpath(genpath(racine));

P     = params();
robot = charger_robot(P);

R1 = scenario1_sans_obstacle(P, robot);
R2 = scenario2_obstacle(P, robot);       % a venir
% R3 = scenario3_singularite(P, robot);    % a venir