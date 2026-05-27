function [trajectory_bus, target_speed, parking_status, selector_ctrl] = parking_scenario_fcn(ego_state, mission, environment, parking_submode)
%#codegen
%% Dev-D | Lib_Parking (100ms)
%% Park the vehicle from Parking_Start to Parking_Goal.
%%
%% OUTPUTS:
%%   trajectory_bus  : 100x3 [x y yaw]
%%   target_speed    : m/s
%%   parking_status  : int32, 0=approach, 1=maneuvering, 2=parked, 9=fail
%%   selector_ctrl   : int32, gear lever
%%                       1  = D (forward maneuver)
%%                      -1  = R (reverse maneuver)
%%                       0  = N (idle/transition)
%%                      -9  = P (parked complete)

%% TODO: implement A* + smoothing from Day4_5
%% TODO: switch selector_ctrl based on parking phase
%%   - approach        : 1  (D)
%%   - reverse maneuver: -1 (R)
%%   - parked          : -9 (P)

trajectory_bus = zeros(100, 3);
target_speed   = 1.0;
parking_status = int32(0);
selector_ctrl  = int32(1);  %% D (forward) — placeholder, Dev-D adjusts by phase
end
