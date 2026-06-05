% ===== OFFLINE REPLAN: REAL drivable surface + REAL parking slot =====
n=200; b=[-5 48 -45.5 6]; resx=(b(2)-b(1))/n; resy=(b(4)-b(3))/n;
xc=b(1)+((1:n)-0.5)*resx; yc=b(3)+((1:n)-0.5)*resy; [XC,YC]=meshgrid(xc,yc);
em=0.20;
Vroad = (XC>=-2.5+em & XC<=2.5-em & YC>=-36.6 & YC<=6);          % x=0 driving road (type0)
Hstrip= (XC>=0      & XC<=47.6-em & YC>=-39.1+em & YC<=-34.1);   % y=-36.6 driving strip (type0)
Bay   = (XC>=5.5+em & XC<=47.6-em & YC>=-34.1 & YC<=-4.1-em);    % type-13 parking bay
BayW  = (XC>=4      & XC<=5.5      & YC>=-34.1 & YC<=-30.1);     % narrow west sliver
M=ones(n,n); M(Vroad|Hstrip|Bay|BayW)=0;
OB=[7.3 -28.7 -1.5708;12.8 -6.8 -1.5708;21.3 -6.6 -1.5708;30 -6.5 -1.5708;41.7 -6.3 -1.5708;31 -33.5 1.5708;7 -21.8 1.5708;18.6 -21.8 1.5708;24.3 -21.9 1.5708;14 -21.9 1.5708;38.8 -21.8 1.5708;41.8 -21.8 1.5708;11 -28.8 -1.5708;24.3 -28.9 -1.5708;41.6 -28.9 -1.5708;15 -33.5 1.5708];
margin=0.40; obs_L=4.47; obs_W=1.97; ahead=obs_L+margin; behind=margin; ohw=obs_W/2+margin;
for k=1:size(OB,1)
  x=OB(k,1);y=OB(k,2);th=OB(k,3); co=cos(th);si=sin(th); lx=[ahead;ahead;-behind;-behind]; ly=[ohw;-ohw;-ohw;ohw];
  c=[x+co*lx-si*ly, y+si*lx+co*ly]; e=1e-9; pos=true(n,n); neg=true(n,n);
  for kk=1:4, k2=mod(kk,4)+1; ex2=c(k2,1)-c(kk,1); ey2=c(k2,2)-c(kk,2); sj=ex2*(YC-c(kk,2))-ey2*(XC-c(kk,1)); pos=pos&(sj>=-e); neg=neg&(sj<=e); end
  M(pos|neg)=1;
end
w=1; M(1:w,:)=1;M(n-w+1:n,:)=1;M(:,1:w)=1;M(:,n-w+1:n)=1;
occ=M;
cfg=pp_cfg(); d_r=cfg.d_r; MAXP=cfg.max_path;
ego_x=-0.294; ego_y=-34.512; ego_yaw=deg2rad(-52.9); fin=[21.45 -21.8]; gth=pi/2;
[ex,ey]=pp_axle_from_fr1(ego_x,ego_y,ego_yaw,d_r); eth=ego_yaw;
[gx,gy]=pp_axle_from_fr1(fin(1),fin(2),gth,d_r);
occ_plan=pp_clear_goal(occ,fin(1),fin(2),gth,cfg);
Dst=cfg.stage_dist; sbx=gx-Dst*cos(gth); sby=gy-Dst*sin(gth); sfx=gx+Dst*cos(gth); sfy=gy+Dst*sin(gth);
okB=~pp_collision(occ_plan,sbx,sby,gth,cfg.foot_ahead,cfg.foot_behind,cfg.veh_halfW,cfg.bounds,cfg.n);
okF=~pp_collision(occ_plan,sfx,sfy,gth,cfg.foot_ahead,cfg.foot_behind,cfg.veh_halfW,cfg.bounds,cfg.n);
dB=hypot(sbx-ex,sby-ey); dF=hypot(sfx-ex,sfy-ey);
tic; okp=false; np=0; dirv=1; px=zeros(MAXP,1);py=px;pth=px;pdir=px;
if okB && (~okF || dB<=dF)
  [px,py,pth,pdir,np,okp]=pp_hybrid_astar(ex,ey,eth,sbx,sby,gth,occ_plan); dirv=1;
  if ~okp && okF, [px,py,pth,pdir,np,okp]=pp_hybrid_astar(ex,ey,eth,sfx,sfy,gth,occ_plan); dirv=-1; end
elseif okF
  [px,py,pth,pdir,np,okp]=pp_hybrid_astar(ex,ey,eth,sfx,sfy,gth,occ_plan); dirv=-1;
  if ~okp && okB, [px,py,pth,pdir,np,okp]=pp_hybrid_astar(ex,ey,eth,sbx,sby,gth,occ_plan); dirv=1; end
end
staged=okp;
if okp, [px,py,pth,pdir,np]=pp_append_straight(px,py,pth,pdir,np,gx,gy,gth,cfg.ds,MAXP,dirv);
else, [px,py,pth,pdir,np,okp]=pp_hybrid_astar(ex,ey,eth,gx,gy,gth,occ_plan); end
tplan=toc;
fprintf('okB=%d okF=%d staged=%d okp=%d np=%d tplan=%.1fs\n',okB,okF,staged,okp,np,tplan);
if np>0, fprintf('path X[%.2f,%.2f] Y[%.2f,%.2f]\n',min(px(1:np)),max(px(1:np)),min(py(1:np)),max(py(1:np))); end
save('D:\HL_IVS_School\Code\motionplanning\26HL_IVS_ADAS\02_Carmaker_project\Practice_sample\src_cm4sl\Models\FinalProject\replan_real_result.mat','occ','px','py','pdir','np','okp','staged','ex','ey','eth','gx','gy','gth','fin','ego_x','ego_y','ego_yaw','xc','yc','OB');
disp('DONE_PLAN');

function [desired_ax, steer_fl, steer_fr, selector_ctrl, vc_gas, vc_brake] = Parking(ego_x, ego_y, ego_yaw, ego_v, start_point, finish_point, goal_yaw, occ_map)
%#codegen
[desired_ax, steer_fl, steer_fr, selector_ctrl, ~, ~, ~, ~, vc_gas, vc_brake] = pp_parking(ego_x, ego_y, ego_yaw, ego_v, start_point, finish_point, goal_yaw, occ_map);
end

function [desired_ax, steer_fl, steer_fr, selector_ctrl, oPX, oPY, oNP, debug, ovc_gas, ovc_brake] = pp_parking(ego_x, ego_y, ego_yaw, ego_v, start_point, finish_point, goal_yaw, occ_map) %#ok<INUSD>
%#codegen
% PP_PARKING  Full precise-parking controller (M10 integration).
%   Supervisor: INIT -> PLAN (Hybrid A*+RS, once) -> TRACK (pure pursuit) ->
%   CORRECT (a few committed Reeds-Shepp corrective rounds, each tracked by
%   pp_track) -> PARKED. Using committed RS paths (fixed cusps) instead of
%   per-step RS feedback avoids gear-shift chatter on the real plant.
%   Gear FSM shifts only when (nearly) stopped. Longitudinal command is in the
%   DRIVING (gear) direction (CarMaker AccelCtrl convention) with an over-speed
%   guard. Ackermann splits the steer into L/R wheels.
%
%   Inputs (Car.Fr1 = rear-bumper center): ego_x,ego_y[m], ego_yaw, ego_v[m/s],
%   finish_point=[x y] (rear bumper), goal_yaw[rad], occ_map. start_point unused.
cfg = pp_cfg(); d_r = cfg.d_r; L = cfg.wheelbase; tw = cfg.track_f; MAXP = cfg.max_path;

persistent state PX PY PTH PDIR NP seg gear replan nround best_perr
if isempty(state)
    state = uint8(0);
    PX = zeros(MAXP,1); PY = zeros(MAXP,1); PTH = zeros(MAXP,1); PDIR = zeros(MAXP,1);
    NP = 0; seg = 1; gear = cfg.GEAR_P; replan = false; nround = 0; best_perr = 1e9;
end

[ex, ey] = pp_axle_from_fr1(ego_x, ego_y, ego_yaw, d_r); eth = ego_yaw;
gth = goal_yaw;
[gx, gy] = pp_axle_from_fr1(finish_point(1), finish_point(2), gth, d_r);

delta = 0.0; v_des = 0.0; hold = false;

switch state
    case uint8(0)                                       % INIT
        state = uint8(1); hold = true;

    case uint8(1)                                       % PLAN (one heavy step)
        occ_plan = pp_clear_goal(occ_map, finish_point(1), finish_point(2), gth, cfg);  % 목표슬롯 차량 제거
        % [전략] 통로(staging)에서 헤딩 정렬 -> 슬롯으로 직진 진입 (비스듬한 진입에 의한 측면차 충돌 방지)
        Dst = cfg.stage_dist;
        sbx = gx - Dst*cos(gth); sby = gy - Dst*sin(gth);            % 뒤쪽 staging (전진 pull-in)
        sfx = gx + Dst*cos(gth); sfy = gy + Dst*sin(gth);            % 앞쪽 staging (후진 reverse-in)
        px = zeros(MAXP,1); py = zeros(MAXP,1); pth = zeros(MAXP,1); pdir = zeros(MAXP,1);  % codegen: 항상 정의
        okp = false; np = 0; dirv = 1;
        okB = ~pp_collision(occ_plan, sbx, sby, gth, cfg.foot_ahead, cfg.foot_behind, cfg.veh_halfW, cfg.bounds, cfg.n);
        okF = ~pp_collision(occ_plan, sfx, sfy, gth, cfg.foot_ahead, cfg.foot_behind, cfg.veh_halfW, cfg.bounds, cfg.n);
        dB = hypot(sbx-ex, sby-ey); dF = hypot(sfx-ex, sfy-ey);   % ego에 가까운 staging 우선(traverse 최소 -> 측면차 충돌 방지)
        if okB && (~okF || dB <= dF)
            [px, py, pth, pdir, np, okp] = pp_hybrid_astar(ex, ey, eth, sbx, sby, gth, occ_plan); dirv = 1;
            if ~okp && okF
                [px, py, pth, pdir, np, okp] = pp_hybrid_astar(ex, ey, eth, sfx, sfy, gth, occ_plan); dirv = -1;
            end
        elseif okF
            [px, py, pth, pdir, np, okp] = pp_hybrid_astar(ex, ey, eth, sfx, sfy, gth, occ_plan); dirv = -1;
            if ~okp && okB
                [px, py, pth, pdir, np, okp] = pp_hybrid_astar(ex, ey, eth, sbx, sby, gth, occ_plan); dirv = 1;
            end
        end
        if okp
            [px, py, pth, pdir, np] = pp_append_straight(px, py, pth, pdir, np, gx, gy, gth, cfg.ds, MAXP, dirv);
            PX = px; PY = py; PTH = pth; PDIR = pdir; NP = np; seg = 1; state = uint8(2);
        else
            [px, py, pth, pdir, np, ok] = pp_hybrid_astar(ex, ey, eth, gx, gy, gth, occ_plan);   % fallback: 직접 계획
            if ok
                PX = px; PY = py; PTH = pth; PDIR = pdir; NP = np; seg = 1; state = uint8(2);
            else
                state = uint8(4);
            end
        end
        hold = true;

    case uint8(2)                                       % TRACK main path
        [delta, v_des, seg, done] = pp_track(ex, ey, eth, ego_v, PX, PY, PDIR, NP, seg, cfg);
        if done, state = uint8(3); replan = true; nround = 0; best_perr = hypot(gx-ex, gy-ey); end

    case uint8(3)                                       % CORRECT = PRECISION PARKING (정밀주차)
        % 현재 자세 -> 최종 목표 자세로 짧은 committed Reeds-Shepp 경로를 재생성하고
        % pp_track로 추종, 위치/yaw 오차가 align_pos/align_yaw 만족(또는 max_rounds)까지
        % 반복. fixed-cusp committed RS라 기어 채터링 없음.
        if replan
            [Tr, sLr, ~, okr] = pp_reedsshepp(ex, ey, eth, gx, gy, gth, cfg.Rmin);
            if okr
                [rx, ry, rth, rdir, rns] = pp_rs_sample(ex, ey, eth, Tr, sLr, cfg.Rmin, cfg.ds_align, MAXP);
                if rns >= 2
                    PX = rx; PY = ry; PTH = rth; PDIR = rdir; NP = rns; seg = 1;
                    replan = false; nround = nround + 1;
                else
                    state = uint8(4);
                end
            else
                state = uint8(4);
            end
        end
        if state == uint8(3)
            [delta, v_des, seg, done] = pp_track(ex, ey, eth, ego_v, PX, PY, PDIR, NP, seg, cfg);
            if done
                if (hypot(gx-ex, gy-ey) < cfg.align_pos && abs(pp_angdiff(eth,gth)) < cfg.align_yaw) ...
                        || nround >= cfg.max_rounds
                    state = uint8(4);                   % 수렴 또는 라운드한계 -> 정지 (레퍼런스대로)
                else
                    replan = true;                      % next committed round
                end
            end
        end

    otherwise                                           % PARKED
        hold = true;
end
if state == uint8(4), hold = true; end

% --- outputs: gear FSM + longitudinal (driving-dir) + Ackermann ---
if hold
    selector_ctrl = cfg.GEAR_P;
    desired_ax    = -1.0;
    steer_fl      = 0.0; steer_fr = 0.0;
else
    if abs(ego_v) < cfg.v_stop                          % shift only when ~stopped
        if v_des > 0.02
            gear = cfg.GEAR_D;
        elseif v_des < -0.02
            gear = cfg.GEAR_R;
        end
    end
    selector_ctrl = gear;
    % CarMaker AccelCtrl.DesiredAx = GEAR-DIRECTION acceleration demand (CONFIRMED by
    % per-gear regression of the logs: in D +DesiredAx -> +x accel (slope +0.84); in R
    % +DesiredAx -> -x accel (slope -0.90)). So +DesiredAx = gas in the CURRENT GEAR
    % direction (forward in D, BACKWARD in R); -DesiredAx = brake. PRIMARY control is this
    % gear-direction speed regulation toward |v_des|, tracked by the vehicle's AccelCtrl PI
    % (tuned in the .car: higher i compensates the reverse idle-creep). Reverse uses a LOW
    % target speed (cfg.v_rev). The over-target branch is only a SAFETY NET (large margin)
    % in case the PI lags -- it should rarely fire once the PI holds the low reverse speed.
    if gear == cfg.GEAR_R, gdir = -1.0; else, gdir = 1.0; end
    v_drive = abs(ego_v);                                  % FIX: ego_v가 부호없는 |v|로 입력됨 → 속도크기로 regulate (gdir*ego_v는 후진서 부호반전 → 가속폭주). gear방향은 기어가 보장.
    desired_ax = cfg.kp_v*(abs(v_des) - v_drive);         % + = gas (gear dir), - = brake
    desired_ax = max(min(desired_ax, cfg.ax_max), -cfg.ax_max);
    if v_drive > cfg.v_emax                                % SAFETY NET: genuine runaway only (PI now
        desired_ax = -cfg.ax_brake;                       % regulates normal speed; no mid-maneuver stops)
    end
    [steer_fl, steer_fr] = pp_ackermann(delta, L, tw);
end
% --- VC.Gas/Brake 페달 직접제어 (reverse_test_ 검증값 기반; .car AccelCtrl PI(P0.001/I1) 우회) ---
%   desired_ax(AccelCtrl)는 미사용/debug용으로 잔존. 모델은 VC.Gas/VC.Brake를 씀.
if hold
    ovc_gas = 0.0; ovc_brake = cfg.vc_brake_hold;              % 정차 유지(P기어)
else
    if gear == cfg.GEAR_R, gas_nom = cfg.vc_gas_rev; vnom = cfg.v_rev;
    else,                  gas_nom = cfg.vc_gas_fwd; vnom = cfg.v_fwd; end
    vmag = abs(v_des);
    if vmag < 0.03
        ovc_gas = 0.0; ovc_brake = cfg.vc_brake_stop;          % cusp/goal 멈춤
    else
        ovc_gas = gas_nom * min(1.0, vmag/max(vnom,0.1));      % v_des에 비례
        if gear == cfg.GEAR_R, gfloor = cfg.vc_gas_min_rev; else, gfloor = cfg.vc_gas_min_fwd; end
        if ovc_gas < gfloor, ovc_gas = gfloor; end             % breakaway floor: 정지마찰 극복(목표근처 stuck 방지)
        ovc_brake = 0.0;
        if abs(ego_v) > vmag + cfg.vc_overspeed
            ovc_gas = 0.0; ovc_brake = cfg.vc_brake_reg;       % 과속 감속
        end
    end
end
oPX = PX; oPY = PY; oNP = NP;        % MF5 path_dbg: 계획경로(rear-axle) 노출

% --- debug (고정 14, 함수 끝에서 1회 조립 → hold/PARKED 포함 모든 상태서 유효) ---
path_yaw_error = 0.0; track_err = 0.0;
if NP >= 1 && seg >= 1 && seg <= NP
    path_yaw_error = pp_angdiff(eth, PTH(seg));      % 현 경로점 heading - ego heading
    track_err = hypot(PX(seg)-ex, PY(seg)-ey);       % 현 segment점과 ego 거리(추종오차)
end
debug = zeros(14,1);
debug(1)  = double(state);            % 0=INIT 1=PLAN 2=TRACK 3=CORRECT(정밀주차) 4=PARKED
debug(2)  = double(seg);              % 현재 추종 segment 인덱스
debug(3)  = double(NP);               % 경로 점 개수 (0 = plan 실패!)
debug(4)  = double(gear);             % -9=P -1=R 0=N 1=D
debug(5)  = v_des;                    % 목표 속도(signed)
debug(6)  = desired_ax;               % 종방향 가속도 명령
debug(7)  = delta;                    % bicycle 조향각 [rad]
debug(8)  = double(nround);           % CORRECT 라운드 수
debug(9)  = double(replan);           % replan 플래그
debug(10) = hypot(gx-ex, gy-ey);      % 목표까지 거리 (= PARKED 시 최종 위치오차)
debug(11) = pp_angdiff(eth, gth);     % 목표 yaw 오차 (= PARKED 시 최종 heading오차)
debug(12) = ego_v;                    % 실제 속도
debug(13) = path_yaw_error;
debug(14) = track_err;
end

function [dL, dR] = pp_ackermann(delta, L, tw)
%#codegen
% PP_ACKERMANN  Left/right front road-wheel angles from a single bicycle steer.
%   delta : bicycle (center) steer angle [rad]; L : wheelbase; tw : front track.
%   Inner wheel steers more than the outer (true Ackermann geometry).
if abs(delta) < 1e-4
    dL = 0.0; dR = 0.0;
    return;
end
R  = L / tan(delta);            % signed turn radius at rear-axle center (+ = left)
dL = atan(L / (R - tw/2));      % left  front wheel
dR = atan(L / (R + tw/2));      % right front wheel
end

function d = pp_angdiff(a, b)
%#codegen
% PP_ANGDIFF  Smallest signed angle a-b wrapped to [-pi, pi].
d = mod(a - b + pi, 2*pi) - pi;
end

function [xa, ya] = pp_axle_from_fr1(x, y, yaw, d_r)
%#codegen
% PP_AXLE_FROM_FR1  Rear-axle position from Fr1 (rear-bumper) pose.
%   Rear axle sits d_r ahead of Fr1 along the heading.
xa = x + d_r*cos(yaw);
ya = y + d_r*sin(yaw);
end

function cfg = pp_cfg()
%#codegen
% PP_CFG  Configuration for the Day4/5 precise-parking controller (Kia_EV6).
%   Geometry from Data/Vehicle/Kia_EV6 (Vehicle.OuterSkin & wheel positions).
%
%   Pose convention (CarMaker): Car.Fr1 origin = REAR-BUMPER center on the road,
%   X forward, Y left, yaw = Car.Fr1.rz [rad]. Same convention for the mission
%   goal pose and every obstacle coordinate. Internal planning/tracking uses the
%   REAR AXLE, d_r ahead of Fr1:  rear_axle = Fr1 + d_r*[cos(yaw) sin(yaw)].
%
%   CODEGEN NOTE: build every struct field from LOCAL variables / literals only.
%   Never read a cfg field while still constructing cfg.

% --- local geometry constants (Kia_EV6) -----------------------------------
veh_L     = 4.68;
veh_W     = 1.88;
wheelbase = 2.90;
d_r       = 0.95;
delta_max = 30*pi/180;

% --- vehicle ---------------------------------------------------------------
cfg.veh_L       = veh_L;
cfg.veh_W       = veh_W;
cfg.veh_halfW   = veh_W/2;
cfg.wheelbase   = wheelbase;
cfg.track_f     = 1.624;                  % front track width [m] (wheels at y=+-0.812)
cfg.d_r         = d_r;
cfg.foot_ahead  = veh_L - d_r;            % rear-axle -> front bumper [m] = 3.73
cfg.foot_behind = d_r;                    % rear-axle -> rear bumper  [m] = 0.95
cfg.delta_max   = delta_max;
cfg.Rmin        = wheelbase / tan(delta_max);

% --- obstacle vehicle (IPG_CompanyCar_2018_Blue) ---------------------------
cfg.obs_L       = 4.47;
cfg.obs_W       = 1.97;
cfg.holdout     = 0;                     % pp_obstacles row kept FREE as the parking
                                          %   target (held-out car). 14 = T13. 0 = none.

% --- occupancy grid --------------------------------------------------------
cfg.n           = 200;                    % cells/side (occ_map [200 200]) -- keep 200: EML occ port is fixed
cfg.margin      = 0.40;                   % obstacle footprint safety inflation [m] (buffer for tracking error)
cfg.bounds      = [-5 48 -45.5 6];        % world planning rect [xmin xmax ymin ymax] = lot + NORTH entrance approach
                                          %   (res 53/200=0.265 x 51.5/200=0.258 m)
cfg.lot         = [2 48 -45.5 -3];        % drivable parking-lot rectangle [xmin xmax ymin ymax] (carved FREE by pp_generate_map)
cfg.edge_margin = 0.20;                    % safety margin inset from the drivable-region boundary (lot edges + corridor walls) [m]
cfg.wall_cells  = 1;                      % occupied border-ring thickness [cells]

% --- Hybrid A* planner -----------------------------------------------------
cfg.plan_res    = 0.5;                    % [튜닝] closed-set 위치해상도[m]. 작게=정밀·느림 / 크게=빠름·거침
cfg.nxc         = 106;                    % closed-set X bins  ~ (bounds_x)/plan_res = 53/0.5
cfg.nyc         = 103;                    % closed-set Y bins  ~ (bounds_y)/plan_res = 51.5/0.5
cfg.nth         = 72;                     % [튜닝] heading bin 개수. 크게=heading 정밀↑·계산량↑
cfg.ds          = 0.6; cfg.ds_align = 0.15;                    % [튜닝] motion step(arc)[m]. 작게=충돌검사/경로정밀↑·계산량↑
cfg.n_steer     = 5;                      % steering samples (odd, symmetric)
cfg.max_nodes   = 60000;                  % expansion / node cap
cfg.max_path    = 2000;                   % max stored path points
cfg.pos_tol     = 0.7;                    % planner goal position tol [m] (M9 refines)
cfg.yaw_tol     = 12*pi/180;              % planner goal heading tol [rad]
cfg.w_rev       = 2.0;                    % reverse cost multiplier
cfg.w_switch    = 2.0;                    % direction-change penalty [~m]
cfg.w_steer     = 0.3;                    % steering magnitude penalty
cfg.h_weight    = 1.2;                    % [튜닝] heuristic 가중(>=1). 크게=goal방향 탐색강화·빨라짐 but 최적성↓
cfg.rs_shot_dist = 15.0;                  % try RS one-shot to goal within this dist [m]
cfg.rs_heur_dist = 25.0;                  % use RS (vs Euclidean) heuristic within [m]
cfg.stage_dist  = 5.0;                     % [전략] 슬롯 직진진입용 staging 거리[m]: 목표서 heading 반대로 이만큼 뒤(통로)서 정렬후 직진

% --- M7 path tracking + longitudinal control -------------------------------
cfg.Ld          = 1.5;                    % (legacy) nominal lookahead [m]
cfg.Ld_min      = 0.5;                    % [튜닝] Pure Pursuit 최소 lookahead[m]. 작게=타이트추종 but 조향진동 가능
cfg.Ld_max      = 1.8;                    % [튜닝] 최대 lookahead[m]. 크게=부드러움 but 코너 크게 돎
cfg.kld         = 1.2;                    % [튜닝] lookahead 속도게인 (Ld=Ld_min+kld*|v|)
cfg.v_fwd       = 0.4;                    % [튜닝] 전진 진입속도[m/s] (0.6->0.4: 더 천천히 정밀 진입)
cfg.v_rev       = 0.2;                    % [튜닝] 후진 목표속도[m/s]. 높이면 빠름 but 주차 정밀/안정성↓
cfg.v_creep     = 0.10;                   % min creep speed while cruising [m/s]
cfg.v_stop      = 0.10;                   % treated as stopped [m/s]
cfg.kp_v        = 0.6;                    % longitudinal P gain (outer speed loop)
cfg.ax_max      = 0.6;                    % accel/brake command limit [m/s^2]
cfg.ax_min      = -1.5;                   % emergency decel/brake limit [m/s^2]
cfg.v_emax      = 1.2;                    % over-speed guard -> emergency brake [m/s]
cfg.v_obrake    = 0.40;                   % SAFETY-NET margin over target -> hard brake [m/s] (PI handles normal regulation)
cfg.ax_brake    = 1.5;                    % safety-net brake demand magnitude [m/s^2] (gear-dir: -ve = brake)
cfg.d_slow      = 5.0;                    % slow-down distance before cusp/goal [m] (weak brake -> decelerate early)
cfg.cusp_reach  = 0.30;                   % distance to register a cusp reached [m]
cfg.goal_reach  = 0.08;                   % path-end position tol [m]
cfg.creep_margin= 0.10;                   % [튜닝] CORRECT중 최근접 대비 이만큼 멀어지면 정지(creep/발산 방지)[m]
cfg.max_rounds  = 8;                      % [튜닝] max RS 보정 라운드 (4→8: 헤딩 수렴 기회↑; 정밀주차)

% --- M9 precise pose alignment (Astolfi pose regulation) -------------------
cfg.align_dist  = 3.0;                    % switch to alignment within this dist [m]
cfg.align_pos   = 0.08;                   % [튜닝] 최종 주차 성공판정 위치오차[m]. 조이면 정밀↑ but 미수렴 위험
cfg.align_yaw   = 2*pi/180;               % [튜닝] 최종 주차 성공판정 heading오차[rad]. 조이면 정밀↑ but 미수렴 위험
cfg.v_align     = 0.3;                    % alignment creep speed [m/s]
cfg.v_align_min = 0.15;                   % min alignment creep speed [m/s]
cfg.kr          = 0.5;                    % Astolfi rho gain (approach)
cfg.ka          = 1.5;                    % Astolfi alpha gain (bearing)
cfg.kb          = -0.5;                   % Astolfi beta gain (final heading, <0)

% --- CarMaker gear selector enum (include/Vehicle.h) -----------------------
cfg.GEAR_P      = -9;
cfg.GEAR_R      = -1;
cfg.GEAR_N      = 0;
cfg.GEAR_D      = 1;

% --- VC.Gas/Brake 페달 직접제어 파라미터 (reverse_test_ 검증값; .car AccelCtrl PI 우회) ---
cfg.vc_gas_fwd    = 0.12;   % 전진 gas (reverse_test_ phase1 검증)
cfg.vc_gas_rev    = 0.30;   % 후진 gas (reverse_test_ phase3 검증; 후진은 더 큰 gas 필요)
cfg.vc_brake_stop = 0.40;   % cusp/goal 멈춤 brake (reverse_test_ 0.35)
cfg.vc_brake_hold = 0.60;   % PARKED/hold 정차 brake (firmer)
cfg.vc_brake_reg  = 0.30;   % 과속 감속 brake
cfg.vc_overspeed  = 0.50;   % |v|>|v_des|+이만큼 빠르면 brake [m/s]
cfg.vc_gas_min_fwd = 0.11;  % 전진 breakaway 최소 gas (정지마찰 극복; 목표근처 stuck 방지)
cfg.vc_gas_min_rev = 0.25;  % 후진 breakaway 최소 gas
end

function occ = pp_clear_goal(occ, gxf, gyf, gyaw, cfg)
%#codegen
% PP_CLEAR_GOAL  동적 holdout: 목표슬롯 차량을 occ에서 제거(0으로).
%   add_obstacle가 traffic의 모든 차(목표위치 차 포함)를 찍으므로, planner가 목표에
%   도달하려면 목표차 footprint를 비워야 함. (gxf,gyf,gyaw)=목표 rear-bumper pose.
%   add_obstacle와 동일 footprint(obs_L/W+margin)로 해당 셀을 free 처리.
b = cfg.bounds; n = cfg.n;
ahead = cfg.obs_L + cfg.margin; behind = cfg.margin; halfw = cfg.obs_W/2 + cfg.margin;   % 목표슬롯 차량 footprint 비움(reverse-in 도달성 위해 원복)
c = pp_rect_corners(gxf, gyf, gyaw, ahead, behind, halfw);
ixv = zeros(1,4); iyv = zeros(1,4);
for k = 1:4
    [ixv(k), iyv(k)] = pp_world2grid(c(k,1), c(k,2), b, n);
end
imin = max(min(ixv),1); imax = min(max(ixv),n);
jmin = max(min(iyv),1); jmax = min(max(iyv),n);
for ix = imin:imax
    for iy = jmin:jmax
        [cx, cy] = pp_grid2world(ix, iy, b, n);
        if pp_pt_in_quad(cx, cy, c)
            occ(iy, ix) = 0;
        end
    end
end
end

function tf = pp_collision(map, x, y, yaw, ahead, behind, halfw, b, n)
%#codegen
% PP_COLLISION  True if the oriented footprint at pose (x,y,yaw) overlaps any
%   occupied cell (map > 0). Pose is the REAR-AXLE reference; the footprint
%   spans [-behind,+ahead] x [-halfw,+halfw]. Scans the footprint bounding box.
c = pp_rect_corners(x, y, yaw, ahead, behind, halfw);
% [보수화] footprint가 planning bounds 밖이면 즉시 충돌 처리. world2grid가 인덱스를
%   [1,n]로 clamp하므로 bounds 밖을 안전영역으로 오판하는 것을 방지. b=[xmin xmax ymin ymax].
if any(c(:,1) < b(1)) || any(c(:,1) > b(2)) || any(c(:,2) < b(3)) || any(c(:,2) > b(4))
    tf = true;
    return;
end
ixv = zeros(1, 4); iyv = zeros(1, 4);
for k = 1:4
    [ixv(k), iyv(k)] = pp_world2grid(c(k,1), c(k,2), b, n);
end
imin = max(min(ixv), 1); imax = min(max(ixv), n);
jmin = max(min(iyv), 1); jmax = min(max(iyv), n);
tf = false;
for ix = imin:imax
    for iy = jmin:jmax
        if map(iy, ix) > 0
            [cx, cy] = pp_grid2world(ix, iy, b, n);
            if pp_pt_in_quad(cx, cy, c)
                tf = true;
                return;
            end
        end
    end
end
end

function [x, y] = pp_grid2world(ix, iy, b, n)
%#codegen
% PP_GRID2WORLD  1-based grid indices -> world coordinates of the CELL CENTER.
%   b = [xmin xmax ymin ymax], n = cells per side.
resx = (b(2) - b(1)) / n;
resy = (b(4) - b(3)) / n;
x = b(1) + (ix - 0.5) * resx;
y = b(3) + (iy - 0.5) * resy;
end

function [px, py, pth, pdir, np, ok] = pp_hybrid_astar(xs, ys, ths, xg, yg, thg, occ_map)
%#codegen
% PP_HYBRID_ASTAR  Hybrid A* parking planner with Reeds-Shepp (REAR-AXLE frame).
%   In : start (xs,ys,ths), goal (xg,yg,thg) [rad], occ_map (n x n, 1=occupied).
%   Out: px,py,pth (max_path x1); pdir (+1 fwd / -1 rev per point); np count; ok.
%   Bicycle fwd/rev primitives + footprint collision; open=min-heap on f=g+h;
%   closed=best-g per (cell,heading). Heuristic = Reeds-Shepp length near goal
%   (Euclidean far). RS one-shot connects to the EXACT goal pose when close.

cfg = pp_cfg();
b = cfg.bounds; n = cfg.n;
plan_res = cfg.plan_res; nxc = cfg.nxc; nyc = cfg.nyc; nth = cfg.nth;
ds = cfg.ds; Lwb = cfg.wheelbase; dmax = cfg.delta_max; Rmin = cfg.Rmin;
ahead = cfg.foot_ahead; behind = cfg.foot_behind; halfw = cfg.veh_halfW;
MAXN = cfg.max_nodes; MAXP = cfg.max_path; ns = cfg.n_steer; hw = cfg.h_weight;
rs_shot = cfg.rs_shot_dist; rs_heur = cfg.rs_heur_dist;
dth_bin = 2*pi/nth;

% steering samples (symmetric about 0)
steer = zeros(1, ns);
for i = 1:ns
    steer(i) = -dmax + (i-1)*(2*dmax/(ns-1));
end

% node + heap + closed storage
NX = zeros(MAXN,1); NY = zeros(MAXN,1); NT = zeros(MAXN,1);
NG = inf(MAXN,1);   NPar = zeros(MAXN,1); NDir = zeros(MAXN,1);
HID = zeros(MAXN,1); HF = inf(MAXN,1); hn = 0;
gbest = inf(nxc, nyc, nth);

% outputs + RS one-shot buffer
px = zeros(MAXP,1); py = zeros(MAXP,1); pth = zeros(MAXP,1); pdir = zeros(MAXP,1);
np = 0; ok = false;
rsx = zeros(MAXP,1); rsy = zeros(MAXP,1); rsth = zeros(MAXP,1); rsdir = zeros(MAXP,1);
rsn = 0; viars = false;

% seed start node
ncount = 1;
NX(1) = xs; NY(1) = ys; NT(1) = ths; NG(1) = 0; NPar(1) = 0; NDir(1) = 0;
six = min(max(floor((xs-b(1))/plan_res)+1,1),nxc);
siy = min(max(floor((ys-b(3))/plan_res)+1,1),nyc);
sit = mod(floor(mod(ths,2*pi)/dth_bin),nth)+1;
gbest(six,siy,sit) = 0;
hn = hn + 1; HID(hn) = 1;
HF(hn) = hw*heur_(xs, ys, ths, xg, yg, thg, Rmin, rs_heur);

goalid = 0; iters = 0;
while hn > 0 && iters < MAXN
    iters = iters + 1;

    % --- heap pop (min f) ---
    cur = HID(1);
    HID(1) = HID(hn); HF(1) = HF(hn); hn = hn - 1;
    hc = 1;
    while true
        hl = 2*hc; hr = 2*hc+1; sm = hc;
        if hl <= hn && HF(hl) < HF(sm), sm = hl; end
        if hr <= hn && HF(hr) < HF(sm), sm = hr; end
        if sm == hc, break; end
        tf = HF(sm); HF(sm) = HF(hc); HF(hc) = tf;
        tk = HID(sm); HID(sm) = HID(hc); HID(hc) = tk;
        hc = sm;
    end

    cx = NX(cur); cy = NY(cur); cth = NT(cur); cg = NG(cur); cdir = NDir(cur);

    % --- tol goal test (fallback) ---
    if abs(cx-xg) < cfg.pos_tol && abs(cy-yg) < cfg.pos_tol && ...
            abs(pp_angdiff(cth,thg)) < cfg.yaw_tol
        goalid = cur; ok = true; break;
    end

    % --- Reeds-Shepp one-shot to exact goal (when close) ---
    if hypot(cx-xg, cy-yg) < rs_shot
        [Tg, segLg, ~, okg] = pp_reedsshepp(cx, cy, cth, xg, yg, thg, Rmin);
        if okg
            [tx, ty, tth, tdir, tn] = pp_rs_sample(cx, cy, cth, Tg, segLg, Rmin, ds, MAXP);
            free = true;
            for sidx = 1:tn
                if pp_collision(occ_map, tx(sidx),ty(sidx),tth(sidx), ahead,behind,halfw, b, n)
                    free = false; break;
                end
            end
            if free
                goalid = cur; ok = true; viars = true; rsn = tn;
                for sidx = 1:tn
                    rsx(sidx)=tx(sidx); rsy(sidx)=ty(sidx); rsth(sidx)=tth(sidx); rsdir(sidx)=tdir(sidx);
                end
                break;
            end
        end
    end

    % --- expand successors (2 directions x ns steers) ---
    for di = 1:2
        if di == 1, dir = 1.0; else, dir = -1.0; end
        sgn = dir*ds;
        for si = 1:ns
            st = steer(si);
            if abs(st) < 1e-6
                nthp = cth;
                nxp = cx + sgn*cos(cth);  nyp = cy + sgn*sin(cth);
                mthp = cth;
                mxp = cx + 0.5*sgn*cos(cth);  myp = cy + 0.5*sgn*sin(cth);
            else
                R = Lwb/tan(st);
                nthp = cth + sgn/R;
                nxp = cx + R*(sin(nthp)-sin(cth));
                nyp = cy - R*(cos(nthp)-cos(cth));
                mthp = cth + 0.5*sgn/R;
                mxp = cx + R*(sin(mthp)-sin(cth));
                myp = cy - R*(cos(mthp)-cos(cth));
            end

            if pp_collision(occ_map, mxp,myp,mthp, ahead,behind,halfw, b, n), continue; end
            if pp_collision(occ_map, nxp,nyp,nthp, ahead,behind,halfw, b, n), continue; end

            stepc = ds;
            if dir < 0, stepc = stepc + ds*(cfg.w_rev-1); end
            if cdir ~= 0 && dir ~= cdir, stepc = stepc + cfg.w_switch; end
            stepc = stepc + cfg.w_steer*abs(st);
            ng = cg + stepc;

            kix = min(max(floor((nxp-b(1))/plan_res)+1,1),nxc);
            kiy = min(max(floor((nyp-b(3))/plan_res)+1,1),nyc);
            kit = mod(floor(mod(nthp,2*pi)/dth_bin),nth)+1;

            if ng < gbest(kix,kiy,kit) && ncount < MAXN
                gbest(kix,kiy,kit) = ng;
                ncount = ncount + 1;
                NX(ncount)=nxp; NY(ncount)=nyp; NT(ncount)=nthp;
                NG(ncount)=ng;  NPar(ncount)=cur; NDir(ncount)=dir;
                fval = ng + hw*heur_(nxp, nyp, nthp, xg, yg, thg, Rmin, rs_heur);
                hn = hn + 1; HID(hn) = ncount; HF(hn) = fval;
                hc = hn;
                while hc > 1
                    hp = floor(hc/2);
                    if HF(hp) <= HF(hc), break; end
                    tf = HF(hp); HF(hp) = HF(hc); HF(hc) = tf;
                    tk = HID(hp); HID(hp) = HID(hc); HID(hc) = tk;
                    hc = hp;
                end
            end
        end
    end
end

% --- reconstruct: Hybrid path (start -> goalid), then append RS one-shot ---
if ok && goalid > 0
    tmpx = zeros(MAXP,1); tmpy = zeros(MAXP,1); tmpt = zeros(MAXP,1); tmpd = zeros(MAXP,1);
    cntr = 0; node = goalid;
    while node ~= 0 && cntr < MAXP
        cntr = cntr + 1;
        tmpx(cntr)=NX(node); tmpy(cntr)=NY(node); tmpt(cntr)=NT(node); tmpd(cntr)=NDir(node);
        node = NPar(node);
    end
    np = cntr;
    for i = 1:cntr
        j = cntr - i + 1;
        px(i)=tmpx(j); py(i)=tmpy(j); pth(i)=tmpt(j); pdir(i)=tmpd(j);
    end
    if cntr >= 2, pdir(1) = pdir(2); end
    if viars
        for sidx = 2:rsn                       % skip RS sample 1 (== node, already added)
            if np >= MAXP, break; end
            np = np + 1;
            px(np)=rsx(sidx); py(np)=rsy(sidx); pth(np)=rsth(sidx); pdir(np)=rsdir(sidx);
        end
    end
end
end

% ===== heuristic: RS length near goal, Euclidean far ======================
function h = heur_(x, y, th, xg, yg, thg, Rmin, rs_heur)
%#codegen
eu = hypot(x-xg, y-yg);
if eu < rs_heur
    [~, ~, Lr, okr] = pp_reedsshepp(x, y, th, xg, yg, thg, Rmin);
    if okr && Lr > eu, h = Lr; else, h = eu; end
else
    h = eu;
end
end

function tf = pp_pt_in_quad(px, py, c)
%#codegen
% PP_PT_IN_QUAD  True if point (px,py) lies inside the convex quad c (4-by-2).
%   Winding-agnostic: inside iff the point is on the same side of all 4 edges.
s = zeros(1, 4);
for k = 1:4
    k2 = mod(k, 4) + 1;
    ex = c(k2,1) - c(k,1);
    ey = c(k2,2) - c(k,2);
    s(k) = ex*(py - c(k,2)) - ey*(px - c(k,1));
end
tf = all(s >= -1e-9) || all(s <= 1e-9);
end

function c = pp_rect_corners(x, y, yaw, ahead, behind, halfw)
%#codegen
% PP_RECT_CORNERS  World corners (4-by-2) of an oriented rectangle.
%   Local frame: X forward, Y left. Rectangle spans X in [-behind, +ahead]
%   and Y in [-halfw, +halfw], placed at pose (x, y, yaw).
%   Corner order: front-left, front-right, rear-right, rear-left (clockwise).
co = cos(yaw); si = sin(yaw);
lx = [ahead,  ahead, -behind, -behind];
ly = [halfw, -halfw, -halfw,   halfw];
c = zeros(4, 2);
for k = 1:4
    c(k,1) = x + co*lx(k) - si*ly(k);
    c(k,2) = y + si*lx(k) + co*ly(k);
end
end

function [T, segLen, Ltot, ok] = pp_reedsshepp(x0, y0, th0, x1, y1, th1, Rmin)
%#codegen
% PP_REEDSSHEPP  Shortest Reeds-Shepp word (CSC+CCC set) between two poses.
%   In : start (x0,y0,th0), goal (x1,y1,th1) [rad], turning radius Rmin [m].
%   Out: T(1x3) segment types (1=L,2=S,3=R); segLen(1x3) signed arc lengths [m]
%        (sign = drive direction); Ltot total length [m]; ok success flag.
dx = x1 - x0;  dy = y1 - y0;
c = cos(th0);  s = sin(th0);
xn = ( c*dx + s*dy) / Rmin;          % goal in start frame, scaled to unit radius
yn = (-s*dx + c*dy) / Rmin;
phi = pp_angdiff(th1, th0);

[T, Lr, Ltot_r, ok] = pp_rs_paths(xn, yn, phi);
segLen = Lr * Rmin;
Ltot   = Ltot_r * Rmin;
end

function [T, L, Ltot, ok] = pp_rs_paths(x, y, phi)
%#codegen
% PP_RS_PATHS  Shortest Reeds-Shepp word for a unit-radius normalized goal
%   (x,y,phi) expressed in the start frame. Covers the CSC + CCC families
%   (LSL,LSR,RSR,RSL,LRL,RLR) via the 3 base words x {id, time-flip, reflect,
%   both} = 12 candidates. Returns:
%     T    : 1x3 segment types (1=L, 2=S, 3=R)
%     L    : 1x3 signed segment lengths in RADIUS units (sign = drive direction)
%     Ltot : total |length| (radius units)
%     ok   : a valid word was found
best = inf;
T = [1 2 1]; L = [0 0 0]; ok = false;

for base = 1:3                 % 1=LSL, 2=LSR, 3=LRL
    for sym = 1:4              % 1=identity, 2=time-flip, 3=reflect, 4=both
        xx = x; yy = y; pp = phi;
        if sym == 2,  xx = -x; yy =  y; pp = -phi; end   % time-flip
        if sym == 3,  xx =  x; yy = -y; pp = -phi; end   % reflect
        if sym == 4,  xx = -x; yy = -y; pp =  phi; end   % both

        [okk, t, u, v] = rs_base(base, xx, yy, pp);
        if ~okk, continue; end

        lens = [t, u, v];
        if sym == 2 || sym == 4, lens = -lens; end        % time-flip negates lengths

        ct = [1 2 1];
        if base == 2, ct = [1 2 3]; elseif base == 3, ct = [1 3 1]; end
        if sym == 3 || sym == 4                           % reflect swaps L<->R
            ct = swapLR(ct);
        end

        tot = abs(lens(1)) + abs(lens(2)) + abs(lens(3));
        if tot < best
            best = tot; T = ct; L = lens; ok = true;
        end
    end
end
Ltot = best;
if ~ok, Ltot = 0; end
end

% ===== base words (PythonRobotics formulation, unit radius) ================
function [ok, t, u, v] = rs_base(base, x, y, phi)
%#codegen
if base == 1
    [ok, t, u, v] = rs_LSL(x, y, phi);
elseif base == 2
    [ok, t, u, v] = rs_LSR(x, y, phi);
else
    [ok, t, u, v] = rs_LRL(x, y, phi);
end
end

function [ok, t, u, v] = rs_LSL(x, y, phi)
%#codegen
ok = false; v = 0;
[u, t] = rs_polar(x - sin(phi), y - 1 + cos(phi));
if t >= 0
    v = rs_mod2pi(phi - t);
    if v >= 0, ok = true; end
end
end

function [ok, t, u, v] = rs_LSR(x, y, phi)
%#codegen
ok = false; t = 0; u = 0; v = 0;
[u1, t1] = rs_polar(x + sin(phi), y - 1 - cos(phi));
u1sq = u1*u1;
if u1sq >= 4
    u = sqrt(u1sq - 4);
    theta = atan2(2.0, u);
    t = rs_mod2pi(t1 + theta);
    v = rs_mod2pi(t - phi);
    if t >= 0 && v >= 0, ok = true; end
end
end

function [ok, t, u, v] = rs_LRL(x, y, phi)
%#codegen
ok = false; t = 0; u = 0; v = 0;
[u1, t1] = rs_polar(x - sin(phi), y - 1 + cos(phi));
if u1 <= 4
    u = -2*asin(u1/4);
    t = rs_mod2pi(t1 + 0.5*u + pi);
    v = rs_mod2pi(phi - t + u);
    if t >= 0 && u <= 0, ok = true; end
end
end

% ===== helpers =============================================================
function [r, th] = rs_polar(a, b)
%#codegen
r = hypot(a, b);
th = atan2(b, a);
end

function v = rs_mod2pi(x)
%#codegen
v = mod(x, 2*pi);
if v > pi, v = v - 2*pi; end
end

function c = swapLR(c)
%#codegen
for k = 1:3
    if c(k) == 1, c(k) = 3; elseif c(k) == 3, c(k) = 1; end
end
end

function [sx, sy, sth, sdir, ns] = pp_rs_sample(x0, y0, th0, T, segLen, Rmin, ds, MAXP)
%#codegen
% PP_RS_SAMPLE  Sample a Reeds-Shepp path into poses for collision checks/output.
%   T(1x3) types (1=L,2=S,3=R), segLen(1x3) signed arc lengths [m].
%   Returns poses sx,sy,sth (MAXP x1), per-point direction sdir (+1/-1), ns count.
sx = zeros(MAXP,1); sy = zeros(MAXP,1); sth = zeros(MAXP,1); sdir = zeros(MAXP,1);
x = x0; y = y0; th = th0;
k = 1; sx(1) = x; sy(1) = y; sth(1) = th; sdir(1) = sign_nz(segLen(1));

for i = 1:3
    Li = segLen(i);
    if abs(Li) < 1e-9, continue; end
    dirn = sign_nz(Li);
    if T(i) == 1
        kappa =  1/Rmin;
    elseif T(i) == 3
        kappa = -1/Rmin;
    else
        kappa = 0;
    end
    dist  = abs(Li);
    steps = max(1, ceil(dist/ds));
    step  = dist/steps;
    for j = 1:steps
        if k >= MAXP, break; end
        sgn = dirn*step;
        if kappa == 0
            x = x + sgn*cos(th);
            y = y + sgn*sin(th);
        else
            th2 = th + sgn*kappa;
            x = x + (sin(th2) - sin(th))/kappa;
            y = y - (cos(th2) - cos(th))/kappa;
            th = th2;
        end
        k = k + 1;
        sx(k) = x; sy(k) = y; sth(k) = th; sdir(k) = dirn;
    end
end
ns = k;
end

function s = sign_nz(v)
%#codegen
if v < 0, s = -1.0; else, s = 1.0; end
end

function [delta, v_des, seg, done] = pp_track(ax, ay, ath, v, PX, PY, PDIR, NP, seg, cfg)
%#codegen
% PP_TRACK  Reverse-aware pure-pursuit path tracker (REAR-AXLE frame).
%   In : ego rear-axle pose (ax,ay,ath), signed speed v, path PX,PY,PDIR(1..NP),
%        current segment index seg, cfg.
%   Out: delta (front road-wheel steer [rad]); v_des (signed target speed [m/s]);
%        seg (advanced, monotonic); done (goal reached).
%   Stays within the current-direction segment (no jumps across cusps), forces a
%   full stop at each cusp before switching direction, and stops at the goal.
L = cfg.wheelbase;
delta = 0; v_des = 0; done = false;
if NP < 2, done = true; return; end
if seg < 1, seg = 1; end
if seg > NP, seg = NP; end

dir = PDIR(seg); if dir == 0, dir = 1; end

% end index of the current-direction segment (cusp = jc, or NP if last)
jc = NP;
for i = seg:NP
    if PDIR(i) ~= dir, jc = i-1; break; end
end
if jc < seg, jc = seg; end

% windowed closest point, clamped to the current segment [seg, jc]
W = 12; hi = min(seg + W, jc);
ci = seg; bestd = inf;
for i = seg:hi
    dxi = PX(i)-ax; dyi = PY(i)-ay;
    di = dxi*dxi + dyi*dyi;
    if di < bestd, bestd = di; ci = i; end
end
seg = ci;

% remaining arc to segment end
remain = 0;
for i = ci:NP-1
    if i >= jc, break; end
    remain = remain + hypot(PX(i+1)-PX(i), PY(i+1)-PY(i));
end
% speed-adaptive lookahead (short at low speed -> tight tracking), capped by remain
Ldspeed = cfg.Ld_min + cfg.kld*abs(v);
Ldeff = min([Ldspeed, cfg.Ld_max, max(remain, cfg.Ld_min)]);
if Ldeff < cfg.Ld_min, Ldeff = cfg.Ld_min; end

% lookahead target within [ci, jc]
ti = min(ci+1, NP); acc = 0; i = ci;
while i < jc && acc < Ldeff
    acc = acc + hypot(PX(i+1)-PX(i), PY(i+1)-PY(i));
    ti = i + 1; i = i + 1;
end
xt = PX(ti); yt = PY(ti);

% pure-pursuit steering (reverse-aware)
bea = atan2(yt-ay, xt-ax);
if dir < 0, head = ath + pi; else, head = ath; end
alpha = pp_angdiff(bea, head);
Ldact = max(hypot(xt-ax, yt-ay), 0.3);
delta = atan2(2*L*sin(alpha), Ldact);
if dir < 0, delta = -delta; end
delta = max(min(delta, cfg.delta_max), -cfg.delta_max);

% speed target: cruise (creep floor), slow approaching the segment end / goal
d_cusp = hypot(PX(jc)-ax, PY(jc)-ay);
d_goal = hypot(PX(NP)-ax, PY(NP)-ay);
dmin = min(d_cusp, d_goal);
if dir > 0, vmax = cfg.v_fwd; else, vmax = cfg.v_rev; end   % faster fwd, slower rev
vmag = max(cfg.v_creep, vmax * min(1.0, dmin/cfg.d_slow));
v_des = dir * vmag;

% stop logic: full stop at a cusp before switching dir, or at the goal
if jc < NP
    if ci >= jc || d_cusp < cfg.cusp_reach
        v_des = 0;
        if abs(v) < cfg.v_stop, seg = jc + 1; end          % switch into next segment
    end
else
    % longitudinal progress toward the final point (handles overshoot)
    p2 = max(NP-1, 1);
    ux = PX(NP)-PX(p2); uy = PY(NP)-PY(p2);
    un = hypot(ux, uy); if un < 1e-6, un = 1; end
    sprog = ((PX(NP)-ax)*ux + (PY(NP)-ay)*uy) / un;
    if d_goal < cfg.goal_reach || sprog <= 0
        v_des = 0;
        if abs(v) < cfg.v_stop, done = true; delta = 0; end
    end
end
end

function [px, py, pth, pdir, np] = pp_append_straight(px, py, pth, pdir, np, gx, gy, gth, ds, MAXP, dirv)
%#codegen
% PP_APPEND_STRAIGHT  Append a straight, heading-aligned segment from the current
%   path end to the goal (gx,gy) at constant heading gth, drive direction dirv
%   (+1 forward pull-in / -1 reverse-in). Car enters the slot already aligned
%   (straight-in), avoiding angled side clipping.
if np < 1
    np = 1; px(1) = gx; py(1) = gy; pth(1) = gth; pdir(1) = dirv;
    return;
end
x0 = px(np); y0 = py(np);
dx = gx - x0; dy = gy - y0;
dist = hypot(dx, dy);
if dist < 1e-6, return; end
nstep = floor(dist / ds);
for i = 1:nstep
    f = (i*ds) / dist;
    if np < MAXP
        np = np + 1;
        px(np) = x0 + f*dx; py(np) = y0 + f*dy; pth(np) = gth; pdir(np) = dirv;
    end
end
if np < MAXP
    np = np + 1; px(np) = gx; py(np) = gy; pth(np) = gth; pdir(np) = dirv;
end
end

function [ix, iy] = pp_world2grid(x, y, b, n)
%#codegen
% PP_WORLD2GRID  World (x,y) -> 1-based grid indices (ix along X/col, iy along Y/row).
%   b = [xmin xmax ymin ymax], n = cells per side. Indices clamped to [1,n].
resx = (b(2) - b(1)) / n;
resy = (b(4) - b(3)) / n;
ix = floor((x - b(1)) / resx) + 1;
iy = floor((y - b(3)) / resy) + 1;
ix = min(max(ix, 1), n);
iy = min(max(iy, 1), n);
end
