function [trajectory_bus, target_speed, tollgate_status, selector_ctrl] = tollgate_scenario_fcn(ego_state, mission, environment, tollgate_submode)
%#codegen
%% Dev-B | Lib_Tollgate (100ms)
%% Pass through hi-pass tollgate lane.
%%
%% OUTPUTS:
%%   trajectory_bus  : 100x3 [x y yaw]
%%   target_speed    : m/s (slow down at tollgate)
%%   tollgate_status : int32, 0=approaching, 1=in-tollgate, 2=passed
%%   selector_ctrl   : int32, gear lever (always 1=D for tollgate)

%% TODO: implement tollgate logic
trajectory_bus  = zeros(100, 3);
target_speed    = 8.0;
tollgate_status = int32(0);
selector_ctrl   = int32(1);  %% D (forward) — tollgate always forward
end
