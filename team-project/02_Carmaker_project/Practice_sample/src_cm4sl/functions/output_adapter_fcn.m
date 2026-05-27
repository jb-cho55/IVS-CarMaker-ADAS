function [steer_cmd_out, accel_cmd_out, selector_ctrl] = output_adapter_fcn(active_scenario, steer_d, accel_d, steer_p, accel_p)
%#codegen
%% Dev-F | Lib_OutputAdapter (10ms)
%% Switch driving/parking controller outputs + saturation.
%%
%% INPUTS: active_scenario (int32), 2 sets of (steer, accel)
%% OUTPUTS: saturated steer_cmd_out, accel_cmd_out, selector_ctrl (DM mode)

if active_scenario == int32(3)  %% PARKING
    steer_cmd_out = steer_p;
    accel_cmd_out = accel_p;
else                            %% DRIVING or TOLLGATE
    steer_cmd_out = steer_d;
    accel_cmd_out = accel_d;
end

%% Saturation (safety)
STEER_MAX = pi/6;     %% rad
ACC_MIN   = -5.0;     %% m/s^2 (brake)
ACC_MAX   = 3.0;      %% m/s^2 (accel)
steer_cmd_out = max(min(steer_cmd_out,  STEER_MAX), -STEER_MAX);
accel_cmd_out = max(min(accel_cmd_out,  ACC_MAX),    ACC_MIN);

selector_ctrl = int32(0);  %% DM Drive mode
end
