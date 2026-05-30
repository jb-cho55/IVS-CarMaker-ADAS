function [desired_ax, steer_fl, steer_fr, path_x_dbg, path_y_dbg, path_len_dbg, selector_ctrl, vc_gas, vc_brake] = ...
    Parking_simple(ego_x, ego_y, ego_yaw, ego_v, start_point, finish_point, goal_yaw, occ_map)
%#codegen
% PLANNER-FREE 전진/후진 제어 검증용 컨트롤러 (MF5 드롭인, RRT* 없음).
%
% 목적: 종방향 제어(가속·감속·정지)와 기어 전환(D->R)을 확실히 검증한다.
%   Phase 0 FORWARD : finish_point 까지 전진 -> 정지
%   Phase 1 REVERSE : start_point  까지 후진 -> 정지
%   Phase 2 PARKED  : 정지 유지(P)
% 두 목표점은 .slx 의 Finish_Point / Start_Point 상수로 사용자가 설정.
%
% 인터페이스 (확정):
%   out1 desired_ax    -> AccelCtrl.DesiredAx [m/s^2]. User 모드에서 차 구동.
%                         +=기어 진행방향 가속, -=감속/브레이크. (기어가 방향 결정)
%   out7 selector_ctrl -> DM.SelectorCtrl  (+1=D, -1=R, -9=P)
%   out2/3 steer_fl/fr -> 앞바퀴 조향각 [rad].  out6 path_len_dbg = 페이즈+1 (Scope 확인)
%   VC.Gas/Brake 미사용.
%
% ★검증 포인트: 후진에서 desired_ax 양수가 '뒤로 가속'이 맞는지 확인.
%   후진이 안 가거나 앞으로 가면 -> REV_AX_SIGN 을 -1.0 으로.
%   후진 조향이 반대로 꺾이면 -> REV_STEER_SIGN 부호 변경.

occ_unused = occ_map(1,1); %#ok<NASGU>
gy_unused  = goal_yaw(1)*0; %#ok<NASGU>
if abs(ego_yaw) > 2.0*pi; ego_yaw = ego_yaw*pi/180.0; end

% ---- 목표점 ----
fwd_tx = finish_point(1); fwd_ty = finish_point(2);   % 전진 목표
rev_tx = start_point(1);  rev_ty = start_point(2);    % 후진 목표

% ---- 튜닝 파라미터 (느리고 확실한 정지 위주) ----
DT        = 0.01;
V_MAX_FWD = 0.6;      % 전진 최고속도 [m/s] (천천히 -> 확실히 정지)
V_MAX_REV = 0.3;      % 후진 최고속도(더 낮게 -> windup/서지 작게)
V_MIN     = 0.30;     % 크립 하한
A_BRAKE   = 0.8;      % 정지거리 프로파일 감속도
AX_MAX_F  = 0.6;      % 전진 가속 캡
AX_MAX_R  = 0.5;      % 후진 가속 캡
AX_MIN    = -2.0;     % 브레이크 캡
KP        = 0.7;      % 전진 속도 P
KP_BRK_R  = 1.5;      % 후진 비례 브레이크 게인(목표 초과 시 즉시 브레이크)
AX_CREEP  = 0.18;     % 후진 가스(목표 이하). 0 금지(stall방지)+낮게(windup천천히).
ARRIVE_TOL= 0.7;      % 목표 도달 반경
STOP_SPEED= 0.08;     % 정지 판정 속도
STOP_MARG = 0.25;
SHIFT_TICKS = int32(50);
SHIFT_VLIM  = 0.12;
K_STEER   = 0.9;
MAX_STEER = 0.5;      % ~28deg
REV_AX_SIGN    = 1.0;  % 후진 desired_ax 부호 = 양수. 확인됨(실측): 양수=기어방향 구동(가스),
                       %   음수=브레이크. 음수로 줬더니 후진을 전혀 안 함(=브레이크라서).
REV_STEER_SIGN = -1.0; % 후진 조향 부호

persistent phase active_selector switch_count stopping init
if isempty(init)
    phase = int32(0);
    active_selector = 1.0;       % 시작 D
    switch_count = int32(0);
    stopping = false;
    init = true;
end

speed_abs = abs(ego_v);

% ---- 현재 페이즈 목표/요청기어 ----
if phase == int32(0)
    tx = fwd_tx; ty = fwd_ty; req_sel = 1.0;
elseif phase == int32(1)
    tx = rev_tx; ty = rev_ty; req_sel = -1.0;
else
    tx = ego_x;  ty = ego_y;  req_sel = -9.0;
end
dist = hypot(tx - ego_x, ty - ego_y);

% ---- 도착 래치: 반경 진입하면 정지모드 고정(살짝 지나쳐도 재가속 금지) ----
if dist < ARRIVE_TOL
    stopping = true;
end
% ---- 페이즈 전이: 정지모드 + 충분히 느림 ----
if phase < int32(2) && stopping && speed_abs < STOP_SPEED
    phase = phase + int32(1);
    stopping = false;
    switch_count = int32(0);
end

% ---- PARKED: 정지 유지 ----
if phase >= int32(2)
    desired_ax = 0.0;
    steer_fl = 0.0; steer_fr = 0.0;
    selector_ctrl = -9.0;            % P
    vc_gas = 0.0; vc_brake = 0.0;
    path_x_dbg = make_dbg(fwd_tx, rev_tx, ego_x);
    path_y_dbg = make_dbg(fwd_ty, rev_ty, ego_y);
    path_len_dbg = int32(3);         % 3 = parked
    return;
end

% ---- 기어 전환(정지 게이트) ----
shifting = false;
tire_angle = 0.0;
if req_sel ~= active_selector
    shifting = true;
    if speed_abs < SHIFT_VLIM
        switch_count = switch_count + int32(1);
        if switch_count > SHIFT_TICKS
            active_selector = req_sel;
            switch_count = int32(0);
            shifting = false;
        end
    else
        switch_count = int32(0);
    end
else
    switch_count = int32(0);
end
selector_ctrl = active_selector;
is_rev = active_selector < 0.0;

% ---- 조향(목표 지향) ----
if ~shifting
    if ~is_rev
        herr = wrap_pi(atan2(ty - ego_y, tx - ego_x) - ego_yaw);
        tire_angle = clamp(K_STEER * herr, -MAX_STEER, MAX_STEER);
    else
        herr = wrap_pi(atan2(ty - ego_y, tx - ego_x) - (ego_yaw + pi));
        tire_angle = clamp(REV_STEER_SIGN * K_STEER * herr, -MAX_STEER, MAX_STEER);
    end
end

% ---- 종방향 속도목표 ----
v_max = V_MAX_FWD; ax_max = AX_MAX_F;
if is_rev; v_max = V_MAX_REV; ax_max = AX_MAX_R; end
if stopping
    v_des = 0.0;                                   % 도착 -> 완전 정지
else
    v_stop = sqrt(2.0 * A_BRAKE * max(dist - STOP_MARG, 0.0));
    v_des = v_max;
    if v_stop < v_des; v_des = v_stop; end
    if v_des < V_MIN; v_des = V_MIN; end           % 크립 하한
end
if shifting; v_des = 0.0; end

% ---- 종방향 명령: 속도 P + anti-stall 플로어 (미분 없음) ----
% 핵심: 내부 AccelCtrl(i=0.1)가 느려서, cruise에서 명령이 0이 되면 차가 코스트->정지
% (특히 후진). 그래서 "목표로 가는 중 + 저속"이면 최소 가스(AX_CREEP)를 유지해 멈추지
% 않게 한다. 목표 반경(stopping) 또는 v_des≈0 이면 브레이크로 확실히 세운다.
% 부호규약: 양수=기어방향 구동(가스), 음수=브레이크.
e = v_des - speed_abs;
if stopping || v_des < 0.05
    desired_ax = -1.0;                       % 정지: 브레이크
    if speed_abs < 0.05; desired_ax = 0.0; end
elseif is_rev
    % ★후진 anti-windup: 양수 DesiredAx를 '지속'하면 AccelCtrl error가 안 닫혀
    % 적분이 감김 -> 급발진. 그래서 히스테리시스로 양수 지속을 피한다:
    %   느림   -> 짧은 가스 nudge
    %   목표속도-> 코스트(0): 정속이면 ActualAx~0 -> error~0 -> 적분 안 감김
    %   과속/서지-> 브레이크: 감긴 적분을 방출
    % 목표 이하면 약한 가스, 목표를 '넘는 즉시' 비례 브레이크(early)로 windup 방출.
    % (직전 FF+P 급발진 원인: 0.15+0.5e 라 e<-0.3 까지 계속 '가스'->windup 누적->서지.
    %  이제 e<0(목표 초과)이면 곧바로 브레이크로 전환 -> windup 안 쌓임. 비례라 부드러움.)
    e = v_des - speed_abs;
    if e >= 0.0
        desired_ax = AX_CREEP;             % 목표 이하 -> 약한 가스(stall 방지)
    else
        desired_ax = KP_BRK_R * e;         % 목표 초과 -> 비례 브레이크(즉시, windup 방출)
        if desired_ax < -1.0; desired_ax = -1.0; end
    end
else
    desired_ax = KP * e;                     % 전진: P (정상 동작)
    if e > 0.0 && speed_abs < 0.30 && desired_ax < AX_CREEP
        desired_ax = AX_CREEP;               % 전진 anti-stall
    end
end
if desired_ax > ax_max; desired_ax = ax_max; end
if desired_ax < AX_MIN; desired_ax = AX_MIN; end

% 후진 부호규약(필요시 반전)
if is_rev
    desired_ax = REV_AX_SIGN * desired_ax;
end

steer_fl = tire_angle;
steer_fr = tire_angle;
vc_gas = 0.0; vc_brake = 0.0;

% 디버그(Scope): [목표1, 목표2, 현재] / path_len_dbg = 페이즈+1 (1=전진,2=후진)
path_x_dbg = make_dbg(fwd_tx, rev_tx, ego_x);
path_y_dbg = make_dbg(fwd_ty, rev_ty, ego_y);
path_len_dbg = phase + int32(1);
end

% ===== local helpers =====
function y = clamp(x, lo, hi)
%#codegen
y = x; if y < lo; y = lo; elseif y > hi; y = hi; end
end

function a = wrap_pi(a)
%#codegen
while a > pi;  a = a - 2.0*pi; end
while a < -pi; a = a + 2.0*pi; end
end

function v = make_dbg(a, b, c)
%#codegen
v = zeros(300,1);
v(1) = a; v(2) = b; v(3) = c;
end
