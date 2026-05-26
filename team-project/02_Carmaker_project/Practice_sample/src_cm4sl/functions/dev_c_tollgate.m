function cte_offset = dev_c_tollgate(ctrl_bus, mode)
%#codegen
% TRAJECTORY_PLANNING_FCN — LC/주차 궤적 보정 (cte_offset 생성)
% Owner: Dev-C (시나리오 1-3 톨게이트)
%
% 4 시나리오:
%   LK   : cte_offset = cte_in (보정 없음)
%   LC_OUT : 0 → −LANE_WIDTH quintic ramp (4초 또는 6초)
%   LC_PASS : −LANE_WIDTH hold
%   TG     : 0 (게이트 차선 중앙)
%   PK     : 0 (TODO: Dev-D 주차 경로 변환)

persistent t_in_state prev_mode
if isempty(t_in_state), t_in_state = 0; prev_mode = int32(1); end

% ===== ctrl_bus 추출 =====
cte_in     = ctrl_bus.cte_in;
ego_x      = ctrl_bus.ego_x;
LANE_WIDTH = ctrl_bus.lane_width;

% ===== 상수 =====
TS         = 0.1;     % 100ms 호출
LC_TIME_OT = 4.0;     % 추월
LC_TIME_TG = 6.0;     % 톨게이트 (저속)
TG_START_X = 500.0;
TG_END_X   = 580.0;

% ===== 모드 진입 시 시간 reset =====
if mode ~= prev_mode
    t_in_state = 0;
    prev_mode = mode;
end

% ===== 모드별 target_offset =====
switch mode
    case int32(3)  % LC_OUT — quintic ramp
        t_in_state = t_in_state + TS;
        in_tg = (ego_x > TG_START_X && ego_x < TG_END_X);
        if in_tg
            LC_TIME = LC_TIME_TG;
        else
            LC_TIME = LC_TIME_OT;
        end
        s = min(1.0, t_in_state / LC_TIME);
        ramp = 10*s^3 - 15*s^4 + 6*s^5;  % C^2 연속
        target_offset = -LANE_WIDTH * ramp;

    case int32(4)  % LC_PASS — hold
        target_offset = -LANE_WIDTH;
        t_in_state = 0;

    case int32(6)  % TOLLGATE — 게이트 중앙
        target_offset = 0;
        t_in_state = 0;

    case int32(7)  % PARKING — TODO
        target_offset = 0;
        t_in_state = t_in_state + TS;

    case int32(0)  % IDLE
        target_offset = 0;
        t_in_state = 0;

    otherwise  % LK, EMERGENCY 등
        target_offset = 0;
        t_in_state = 0;
end

cte_offset = cte_in - target_offset;
end
