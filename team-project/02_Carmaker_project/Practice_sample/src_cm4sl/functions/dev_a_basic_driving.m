function acc_cmd = dev_a_basic_driving(ctrl_bus, mode)
%#codegen
% LONGITUDINAL_CONTROLLER_FCN — 종방향 제어 (ACC + 모드별 분기)
% Owner: Dev-A (시나리오 1-1 기본 주행)
%
% 4 시나리오 게인:
%   LK   : v_target=user, Kp=0.5 (평시)
%   LC_PASS : v_target+=5, Kp=0.8 (추월 가속)
%   TG   : v_target ramp→8, Kp=0.4 (점진 감속)
%   PK   : v_target ramp→2, Kp=0.3 (저속)
%   EMG  : -5 (즉시)
%   IDLE : -1 (정지 유지)

persistent v_int e_prev v_target_smooth prev_mode
if isempty(v_int)
    v_int = 0; e_prev = 0; v_target_smooth = 20; prev_mode = int32(1);
end

% ===== ctrl_bus 추출 =====
v_ego     = ctrl_bus.ego_v;
v_target  = ctrl_bus.v_target;
front_gap = ctrl_bus.front_gap;

% ===== 모드별 v_target_raw + 게인 =====
switch mode
    case int32(9)  % EMERGENCY
        acc_cmd = -5.0; v_int = 0; e_prev = 0;
        v_target_smooth = v_ego; prev_mode = mode; return;

    case int32(4)  % LC_PASS
        v_target_raw = v_target + 5.0;
        Kp = 0.8; Ki = 0.1; Kd = 0.05;

    case int32(6)  % TOLLGATE
        v_target_raw = 8.0;
        Kp = 0.4; Ki = 0.05; Kd = 0.1;

    case int32(7)  % PARKING
        v_target_raw = 2.0;
        Kp = 0.3; Ki = 0.05; Kd = 0.15;

    case int32(0)  % IDLE
        acc_cmd = -1.0; v_int = 0;
        v_target_smooth = 0; prev_mode = mode; return;

    otherwise  % LK (1), LC_OUT (3), 기타
        v_target_raw = v_target;
        Kp = 0.5; Ki = 0.1; Kd = 0.05;
end

% ===== 점진적 v_target 변화 (급가/급감속 방지) =====
% TS=10ms, max rate 2 m/s² → max delta = 0.02 m/s per cycle
MAX_VT_RATE = 0.02;
if mode ~= prev_mode
    v_target_smooth = v_ego;  % 모드 전환 시 현재 속도부터 시작
end
delta = v_target_raw - v_target_smooth;
if delta > MAX_VT_RATE
    v_target_smooth = v_target_smooth + MAX_VT_RATE;
elseif delta < -MAX_VT_RATE
    v_target_smooth = v_target_smooth - MAX_VT_RATE;
else
    v_target_smooth = v_target_raw;
end
prev_mode = mode;

% ===== 안전거리 모델 + 오차 =====
safe_dist = 10.0 + 2.0 * v_ego;
e_d = front_gap - safe_dist;
e_v = v_target_smooth - v_ego;
e_eff = min(e_d, e_v);

% ===== PID + Anti-windup =====
de = e_eff - e_prev;
e_prev = e_eff;
v_int = max(min(v_int + e_eff*0.01, 2.0), -2.0);
acc_cmd = Kp*e_eff + Ki*v_int + Kd*de;

% ===== 비상 제동 =====
if front_gap < 5.0
    acc_cmd = -5.0;
end

% ===== 한계 =====
acc_cmd = max(min(acc_cmd, 3.0), -5.0);
end
