function [active_scenario, driving_submode, tollgate_submode, parking_submode] = mission_manager_fcn(ego_state, mission, driving_status, tollgate_status, parking_status)
%#codegen
%% Dev-F | Lib_MissionManager (10ms)
%% Decide which top-level scenario is active and report sub-modes.
%%
%% INPUTS:
%%   ego_state         : EgoStateBus (Car_Fr1_tx/ty/rz, Ego_V, CrossTrackError, Ego_Vx_Body)
%%   mission           : MissionBus (Waypoints, Parking_Start/Goal_X/Y/Yaw)
%%   driving_status    : int32 from Lib_Driving
%%   tollgate_status   : int32 from Lib_Tollgate
%%   parking_status    : int32 from Lib_Parking
%%
%% OUTPUTS:
%%   active_scenario   : int32  1=DRIVING, 2=TOLLGATE, 3=PARKING
%%   driving_submode   : int32  (Dev-A defines)
%%   tollgate_submode  : int32  (Dev-B defines)
%%   parking_submode   : int32  (Dev-D defines)

%% TODO: Replace placeholder with actual FSM logic
%% Suggested transitions:
%%   DRIVING -> TOLLGATE  when distance to tollgate < threshold
%%   TOLLGATE -> DRIVING  when tollgate passed
%%   DRIVING -> PARKING   when ego near Parking_Start_Point

active_scenario  = int32(1);  %% DRIVING by default
driving_submode  = int32(0);
tollgate_submode = int32(0);
parking_submode  = int32(0);
end
