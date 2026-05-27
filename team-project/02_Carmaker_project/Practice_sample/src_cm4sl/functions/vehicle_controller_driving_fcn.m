function [steer_cmd, accel_cmd] = vehicle_controller_driving_fcn(ego_state, active_scenario, driving_traj, driving_speed, tollgate_traj, tollgate_speed)
%#codegen
%% Dev-C | Lib_VehicleController_Driving (10ms)
%% Track trajectory; switch source between driving and tollgate by active_scenario.
%%
%% INPUTS:
%%   ego_state       : EgoStateBus
%%   active_scenario : int32 (1=DRIVING, 2=TOLLGATE)
%%   driving_traj  / tollgate_traj  : 100x3 trajectory
%%   driving_speed / tollgate_speed : target speed
%%
%% OUTPUTS:
%%   steer_cmd : rad
%%   accel_cmd : m/s^2

%% Select trajectory source by scenario
if active_scenario == int32(2)
    traj   = tollgate_traj;
    v_tgt  = tollgate_speed;
else
    traj   = driving_traj;
    v_tgt  = driving_speed;
end

%% TODO: implement Stanley/Pure Pursuit + PID
steer_cmd = 0.0;
accel_cmd = 0.5;  %% gentle gas placeholder
end
