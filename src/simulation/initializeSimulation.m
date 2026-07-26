function simState = initializeSimulation(mission)
%INITIALIZESIMULATION Initialize simulation state.
%
% Inputs:
%   mission - Mission configuration structure
%
% Outputs:
%   simState - Simulation state structure

simState.currentTime = mission.startTime;

simState.currentPhase = "Pre-Launch";

simState.isMissionRunning = true;

simState.simulationStep = 0;

end