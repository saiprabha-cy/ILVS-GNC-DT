function mission = missionConfig()
%MISSIONCONFIG Load mission configuration parameters.
%
% Project:
%   Integrated Launch Vehicle & Satellite GNC Digital Twin
%
% Description:
%   Defines the global mission configuration used throughout the
%   simulation. This centralizes configurable parameters and avoids
%   hard-coded values elsewhere in the project.

mission.name = "Integrated Launch Vehicle & Satellite GNC Digital Twin";

mission.version = "1.0.0";

mission.author = "SaiPrabha C Y";

mission.startTime = 0;          % seconds

mission.timeStep = 0.1;         % seconds

mission.endTime = 300;          % seconds (temporary)

mission.launchSite = "Generic Launch Site";

mission.vehicle = "Educational Launch Vehicle";

mission.targetOrbit = "Low Earth Orbit";

end