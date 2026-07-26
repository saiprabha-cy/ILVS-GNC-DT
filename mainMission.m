clc;
clear;
close all;

%% Add Project Paths
addpath(genpath('config'));
addpath(genpath('src'));

%% Mission Banner
disp('=======================================================')
disp(' Integrated Launch Vehicle & Satellite GNC Digital Twin ')
disp('=======================================================')

%% Load Mission Configuration
mission = missionConfig();

%% Initialize Simulation
simState = initializeSimulation(mission);

%% Display Mission Information

fprintf('\nMission Name      : %s\n', mission.name);
fprintf('Version           : %s\n', mission.version);
fprintf('Launch Site       : %s\n', mission.launchSite);
fprintf('Target Orbit      : %s\n', mission.targetOrbit);

fprintf('\nSimulation Initialized Successfully.\n');

fprintf('Current Mission Phase : %s\n', simState.currentPhase);
fprintf('Simulation Time       : %.2f seconds\n', simState.currentTime);

disp('-------------------------------------------------------')
disp('System Status : READY FOR LAUNCH')
disp('-------------------------------------------------------')