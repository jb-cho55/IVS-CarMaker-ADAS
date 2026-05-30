function [desired_ax, steer_fl, steer_fr, path_x_dbg, path_y_dbg, path_len_dbg, selector_ctrl, vc_gas, vc_brake] = ...
    Parking_simple(ego_x, ego_y, ego_yaw, ego_v, start_point, finish_point, goal_yaw, occ_map)
%#codegen
% ===========================================================================
%  매뉴얼 기반 최소 주차 제어 (CarMaker Reference Manual §7.2 / §7.3 / §7.4)
% ---------------------------------------------------------------------------
%  종방향: AccelCtrl.DesiredAx (= a_desr, m/s²)를 준다.
%          AccelCtrl PI 가 Δa = a_desr − a_actual 로 VC.Gas/VC.Brake 를 만든다(Fig 7.3).
%          => a_desr 를 '부드럽게' 줘야 PI 가 과반응(surge) 하지 않는다.
%  방향  : DM.SelectorCtrl  (+1 = D 전진 / −1 = R 후진 / −9 = P 주차)
%  부호  : a_desr 양수 = 기어 진행방향 가속(가스), 음수 = 브레이크.
%
%  설계(부드러운 a_desr 만들기):
%    (1) 정지거리 속도 프로파일: v_tgt = sqrt(2·A_DEC·남은거리)  → 목표 가까우면 감속
%    (2) 속도 P:               a_desr = KP·(v_tgt − |v|)
%    (3) a_desr 상승률 제한:    가스(상승)는 완만히 램프 → PI 에 완만한 setpoint(=surge 완화).
%                              브레이크(하강)는 자유 → 빨리 멈춤.
%
%  한계(매뉴얼 구조상): AccelCtrl PI 는 a_actual(차체기준)로 닫혀, 후진 정속 cruise 를
%    못 잡는다(후진 가스 → a_actual 음수 → Δa 커짐 → windup). 따라서
%    - 짧은 후진(가속→감속 한 번)은 매끈,  - 긴 후진(정속 구간 필요)은 펄스 발생 = 정상.
%
%  Phase 0: finish_point 까지 전진(D)
%  Phase 1: start_point  까지 후진(R)
%  Phase 2: 정지(P)
% ===========================================================================
u1 = occ_map(1,1)*0;  %#ok<NASGU>
u2 = goal_yaw(1)*0;   %#ok<NASGU>
if abs(ego_yaw) > 2.0*pi; ego_yaw = ego_yaw*pi/180.0; end

% ---- 파라미터 ----
V_MAX_F    = 0.7;     % 전진 속도 상한 [m/s]
V_MAX_R    = 0.3;     % 후진 속도 상한 [m/s] (후진은 살살 -> 낮게)
A_DEC      = 0.5;     % 정지거리 프로파일 감속도 [m/s^2] (부드러운 정지)
V_MIN      = 0.25;    % 크립 하한(목표 도달 전)
KP         = 0.8;     % 속도 P 게인
AX_MAX     = 0.6;     % 가속 캡
AX_MIN     = -1.5;    % 브레이크 캡
AX_SLEW_UP = 0.05;    % a_desr 상승률 제한(스텝당). 가스 완만 → PI surge 완화. (튜닝 가능)
TOL        = 0.7;     % 목표 도달 반경 [m]
STOP_SPD   = 0.10;    % 정지 판정 속도
STOP_MARG  = 0.25;    % 정지거리 여유
STEER_K_F  = 0.8;     % 전진 조향 P
STEER_K_R  = 0.35;    % 후진 조향 P (낮게 -> 좌우 흔들림 감소)
STEER_MAX  = 0.5;     % 조향 캡 [rad]
HERR_DEAD  = 0.025;   % 조향 헤딩오차 deadband [rad] -> 작은 오차 무시(트위치 방지)
REV_DIST   = 4.0;     % 후진 거리 [m]. 전진 끝지점에서 '짧게' 뒤로(실제 주차처럼). (튜닝 가능)

persistent phase prev_ax rev_tx rev_ty init
if isempty(init)
    phase   = int32(0);
    prev_ax = 0.0;
    rev_tx  = 0.0;
    rev_ty  = 0.0;
    init    = true;
end

% ---- 현재 목표 / 기어 ----
if phase == int32(0)
    tx = finish_point(1); ty = finish_point(2); sel = 1.0;     % 전진: finish_point 까지
elseif phase == int32(1)
    tx = rev_tx; ty = rev_ty; sel = -1.0;                      % 후진: 짧게(REV_DIST) 뒤로
else
    tx = ego_x; ty = ego_y; sel = -9.0;                        % 주차 P
end
dist  = hypot(tx - ego_x, ty - ego_y);
speed = abs(ego_v);

% ---- 페이즈 전이: 목표 도달 + 정지 ----
if phase < int32(2) && dist < TOL && speed < STOP_SPD
    if phase == int32(0)
        % 후진 목표 = 전진 끝지점에서 REV_DIST 만큼 '뒤로'(차 뒤쪽). 먼 start_point 대신 짧은 후진.
        rev_tx = ego_x - REV_DIST * cos(ego_yaw);
        rev_ty = ego_y - REV_DIST * sin(ego_yaw);
    end
    phase = phase + int32(1);
end

% ---- 조향 (목표 지향, 전진/후진 분리) ----
if sel >= 0.0
    herr = wrap_pi(atan2(ty - ego_y, tx - ego_x) - ego_yaw);          % 전진: 앞이 목표 향함
    sk = STEER_K_F;
else
    herr = -wrap_pi(atan2(ty - ego_y, tx - ego_x) - (ego_yaw + pi));  % 후진: 뒤가 목표 향함
    sk = STEER_K_R;                                                   % 후진 낮은 게인 -> wobble 감소
end
if herr < HERR_DEAD && herr > -HERR_DEAD; herr = 0.0; end             % 작은 오차 무시(트위치 방지)
steer = sk * herr;
if steer >  STEER_MAX; steer =  STEER_MAX; end
if steer < -STEER_MAX; steer = -STEER_MAX; end
if sel < -1.5; steer = 0.0; end

% ---- 종방향: 정지거리 속도프로파일 → 속도 P → a_desr ----
v_max = V_MAX_F;
if sel < 0.0; v_max = V_MAX_R; end                     % 후진은 살살(낮은 상한)
v_tgt = sqrt(2.0 * A_DEC * max(dist - STOP_MARG, 0.0));
if v_tgt > v_max; v_tgt = v_max; end
if dist > TOL && v_tgt < V_MIN; v_tgt = V_MIN; end     % 크립 하한
if phase >= int32(2) || dist <= TOL; v_tgt = 0.0; end  % 도달/주차 → 정지
desired_ax = KP * (v_tgt - speed);                     % >0 가속(기어방향), <0 브레이크
if desired_ax > AX_MAX; desired_ax = AX_MAX; end
if desired_ax < AX_MIN; desired_ax = AX_MIN; end

% a_desr 상승률 제한: 가스(상승)만 완만히 → AccelCtrl PI 에 완만한 setpoint(=surge 완화).
% 브레이크(하강)는 제한 없음 → 빨리 멈춤/windup 방출.
if desired_ax > prev_ax + AX_SLEW_UP
    desired_ax = prev_ax + AX_SLEW_UP;
end
prev_ax = desired_ax;

% ---- 출력 ----
selector_ctrl = sel;
steer_fl = steer;
steer_fr = steer;
vc_gas = 0.0; vc_brake = 0.0;
path_x_dbg = mk_dbg(finish_point(1), start_point(1), ego_x);
path_y_dbg = mk_dbg(finish_point(2), start_point(2), ego_y);
path_len_dbg = phase + int32(1);   % 1=전진, 2=후진, 3=주차

end

% ===== helpers =====
function a = wrap_pi(a)
%#codegen
while a >  pi; a = a - 2.0*pi; end
while a < -pi; a = a + 2.0*pi; end
end

function v = mk_dbg(a, b, c)
%#codegen
v = zeros(300,1);
v(1) = a; v(2) = b; v(3) = c;
end
