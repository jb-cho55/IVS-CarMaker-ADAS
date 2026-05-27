function [steer_cmd, accel_cmd] = vehicle_controller_parking_fcn(ego_state, parking_traj, parking_speed)
%#codegen
%% Dev-E | Lib_VehicleController_Parking (10ms)
%% Low-speed precise tracking for parking maneuver.
%%
%% INPUTS: ego_state, parking_traj (100x3), parking_speed (m/s, signed)
%% OUTPUTS: steer_cmd (rad), accel_cmd (m/s^2)

%% TODO: implement parking controller
%% - Smaller lookahead for precision
%% - Reverse-aware (sign of parking_speed)
%% - Larger steering authority

steer_cmd = 0.0;
accel_cmd = 0.0;
end
