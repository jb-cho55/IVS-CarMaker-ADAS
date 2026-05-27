function [trajectory_bus, target_speed, parking_status] = parking_scenario_fcn(ego_state, mission, environment, parking_submode)
%#codegen
%% Dev-D | Lib_Parking (100ms)
%% Park the vehicle from Parking_Start to Parking_Goal.
%%
%% INPUTS: 3 buses + sub-mode
%% OUTPUTS:
%%   trajectory_bus  : 100x3 [x y yaw] (may include reverse motion)
%%   target_speed    : m/s (signed: + forward, - reverse)
%%   parking_status  : int32, 0=approach, 1=maneuvering, 2=parked, 9=fail

%% TODO: implement A* + smoothing from Day4_5
%% - Parking_Start_Point from mission.Parking_Start_Point_X/Y
%% - Parking_Goal_Point  from mission.Parking_Goal_Point_X/Y/Yaw
%% - Map boundary from environment.MapBoundary
%% - Obstacles from environment.ObstacleInfo

trajectory_bus = zeros(100, 3);
target_speed   = 1.0;   %% m/s, slow parking speed
parking_status = int32(0);
end
