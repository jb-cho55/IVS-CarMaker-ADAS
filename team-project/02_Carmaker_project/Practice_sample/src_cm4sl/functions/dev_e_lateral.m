function steer_cmd = dev_e_lateral(ctrl_bus, mode, cte)
%#codegen
% LATERAL_CONTROLLER_FCN — 횡방향 제어 (PID + 모드별 게인)
% Owner: Dev-E (Cross-cutting, 4 시나리오 모두 횡 제어)
%
% Inputs:
%   ctrl_bus : Supervisor 정리된 신호 (현재는 ego_v 정도 활용 — 향후 속도 적응 게인 등)
%   mode     : Mission_Mode (int32)
%   cte      : Trajectory가 보정한 Cross-Track Error [m]
% Output:
%   steer_cmd : 앞바퀴 조향각 [rad], 범위 ±π/6

persistent e_prev e_int
if isempty(e_prev), e_prev = 0; e_int = 0; end

% ===== 모드별 게인 =====
switch mode
    case int32(1)  % LK
        Kp = 1.5; Ki = 0.3; Kd = 0.4;
    case {int32(3), int32(4)}  % LC_OUT, LC_PASS
        Kp = 2.0; Ki = 0.5; Kd = 0.3;
    case int32(6)  % TOLLGATE
        Kp = 1.0; Ki = 0.2; Kd = 0.8;
    case int32(7)  % PARKING
        Kp = 0.8; Ki = 0.2; Kd = 0.9;
    case int32(0)  % IDLE
        steer_cmd = 0; e_prev = 0; e_int = 0; return;
    case int32(9)  % EMERGENCY
        Kp = 0.8; Ki = 0.1; Kd = 0.5;
    otherwise
        Kp = 1.5; Ki = 0.3; Kd = 0.4;
end

% ===== 속도 적응 게인 (고속에서 게인 ↓ — 과조향 방지) =====
v_ego = ctrl_bus.ego_v;
if v_ego > 15
    speed_factor = 15 / v_ego;  % 고속일수록 게인 작게
    Kp = Kp * speed_factor;
end

% ===== PID =====
e  = -cte;  % 부호 규칙: DevDist>0 (오른쪽 벗어남) → steer>0 (좌조향)
de = e - e_prev;
e_int = max(min(e_int + e*0.01, 3.0), -3.0);
e_prev = e;

steer_cmd = Kp*e + Ki*e_int + Kd*de;

% ===== 조향각 한계 =====
steer_cmd = max(min(steer_cmd, pi/6), -pi/6);
end
