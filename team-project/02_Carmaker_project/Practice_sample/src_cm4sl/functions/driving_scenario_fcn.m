function [trajectory_bus, target_speed, driving_status, selector_ctrl] = driving_scenario_fcn(ego_state, mission, environment, driving_submode)
%#codegen
%% Dev-A | Lib_Driving (100ms)
%% Cruise + Overtake: decide path + speed for general driving.
%%
%% INPUTS:
%%   ego_state, mission, environment : 3 buses from Lib_Supervisor
%%   driving_submode                 : int32 from Lib_MissionManager
%%
%% OUTPUTS:
%%   trajectory_bus : 100x3 matrix [x y yaw] target path
%%   target_speed   : double, m/s
%%   driving_status : int32, 0=normal, 1=overtaking, ...
%%   selector_ctrl  : int32, gear lever (always 1=D for driving)

%% TODO: implement cruise + overtake
trajectory_bus = zeros(100, 3);
target_speed   = 20.0;
driving_status = int32(0);
selector_ctrl  = int32(1);  %% D (forward) — driving always forward
end
