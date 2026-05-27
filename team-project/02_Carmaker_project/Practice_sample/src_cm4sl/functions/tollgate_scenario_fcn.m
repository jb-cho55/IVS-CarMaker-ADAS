function [trajectory_bus, target_speed, tollgate_status] = tollgate_scenario_fcn(ego_state, mission, environment, tollgate_submode)
%#codegen
%% Dev-B | Lib_Tollgate (100ms)
%% Pass through hi-pass tollgate lane.
%%
%% INPUTS: 3 buses from Supervisor + sub-mode
%% OUTPUTS:
%%   trajectory_bus  : 100x3 [x y yaw]
%%   target_speed    : m/s (slow down at tollgate)
%%   tollgate_status : int32, 0=approaching, 1=in-tollgate, 2=passed

%% TODO: implement tollgate logic
%% - Identify hi-pass lane from waypoints (specific LaneID)
%% - Lateral lane change to hi-pass
%% - Speed profile (typically <30km/h at tollgate)

trajectory_bus  = zeros(100, 3);
target_speed    = 8.0;   %% m/s ~ 30km/h
tollgate_status = int32(0);
end
