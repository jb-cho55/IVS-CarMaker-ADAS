function mode = dev_b_overtaking(ctrl_bus, target_lane)
%#codegen
% MODE_MANAGER_FCN — 미션 모드 결정 (State Machine)
% Owner: Dev-B (시나리오 1-2 추월)
%
% 4 시나리오 지원:
%   1-1. 기본 주행  : LK (1)
%   1-2. 추월       : LK(1) → LC_OUT(3) → LC_PASS(4) → LK(1) (옆차선 유지)
%   1-3. 톨게이트   : LK(1) → TG(6) → LC_OUT(3) → TG(6) → LK(1)
%   2-1. 주차       : LK(1) → PARKING(7) → IDLE(0)
%
% Inputs:
%   ctrl_bus    : Supervisor가 만든 정리된 신호 다발 (struct)
%   target_lane : path_planning이 추천한 차선 (int32)
% Output:
%   mode : int32
%     0=IDLE 1=LK 3=LC_OUT 4=LC_PASS 6=TG 7=PARKING 9=EMERGENCY

persistent state t_in_state
if isempty(state), state = int32(1); t_in_state = 0; end

% ===== ctrl_bus 필드 추출 =====
ego_v        = ctrl_bus.ego_v;
ego_x        = ctrl_bus.ego_x;
long_dist    = ctrl_bus.long_dist;
lat_dist     = ctrl_bus.lat_dist;
current_lane = ctrl_bus.current_lane;

% ===== 시나리오별 상수 =====
TS = 0.01;
TG_START_X      = 500.0;   % Dev-C 시나리오
TG_END_X        = 580.0;
PK_START_X      = 600.0;   % Dev-D 시나리오
PK_START_Y      = 50.0;
PK_GOAL_X       = 580.0;
PK_GOAL_Y       = 100.0;
PK_TRIGGER_DIST = 30.0;
PK_DONE_DIST    = 1.5;
LC_DURATION     = 4.0;
OT_TRIGGER_DIST = max(40.0, ego_v * 3.0);
OT_PASS_DIST    = -15.0;
EMG_DIST        = 5.0;

% ===== 주차 거리 =====
dist_pk_start = sqrt((ego_x - PK_START_X)^2 + (ctrl_bus.ego_y - PK_START_Y)^2);
dist_pk_goal  = sqrt((ego_x - PK_GOAL_X)^2  + (ctrl_bus.ego_y - PK_GOAL_Y)^2);

t_in_state = t_in_state + TS;

% ===== EMERGENCY (최우선) =====
if long_dist > 0 && long_dist < EMG_DIST && abs(lat_dist) < 1.5 && ego_v > 5
    state = int32(9); mode = state; return;
end

% ===== State Machine =====
switch state
    case int32(1)  % LK
        if dist_pk_start < PK_TRIGGER_DIST
            state = int32(7); t_in_state = 0;             % 주차 진입
        elseif ego_x > TG_START_X && ego_x < TG_END_X
            state = int32(6); t_in_state = 0;             % 톨게이트 진입
        elseif target_lane ~= current_lane && ego_v > 5 && ...
               long_dist > 0.5 && long_dist < OT_TRIGGER_DIST && abs(lat_dist) < 1.8
            state = int32(3); t_in_state = 0;             % 추월 LC 진입
        end

    case int32(3)  % LC_OUT
        if t_in_state >= LC_DURATION
            if ego_x > TG_START_X && ego_x < TG_END_X
                state = int32(6); t_in_state = 0;         % TG로 복귀
            else
                state = int32(4); t_in_state = 0;         % 추월 가속
            end
        end

    case int32(4)  % LC_PASS (추월 가속)
        if long_dist < OT_PASS_DIST || long_dist > 150
            state = int32(1); t_in_state = 0;             % 옆차선 유지
        end

    case int32(6)  % TOLLGATE
        if target_lane ~= current_lane
            state = int32(3); t_in_state = 0;             % LC for gate
        elseif ego_x > TG_END_X
            state = int32(1); t_in_state = 0;             % TG 탈출
        end

    case int32(7)  % PARKING
        if dist_pk_goal < PK_DONE_DIST && ego_v < 0.5
            state = int32(0); t_in_state = 0;             % 주차 완료
        end

    case int32(9)  % EMERGENCY
        if long_dist > EMG_DIST * 3 || long_dist < 0
            state = int32(1); t_in_state = 0;
        end

    case int32(0)  % IDLE
        if ego_v > 1.0
            state = int32(1); t_in_state = 0;
        end
end

mode = state;
end
