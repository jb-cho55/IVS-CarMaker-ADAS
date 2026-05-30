function [desired_ax, steer_fl, steer_fr, path_x_dbg, path_y_dbg, path_len_dbg, selector_ctrl, vc_gas, vc_brake] = Parking(ego_x, ego_y, ego_yaw, ego_v, start_point, finish_point, goal_yaw, occ_map)
%PARKING  Self-contained planner + controllers for the Day4_5_Scenario_1.slx
%   "MATLAB Function" (Parking) block.
%
%   RRT* + Reeds-Shepp (CCRS) planner -> Stanley lateral + PD longitudinal.
%
%   Inputs
%       ego_x, ego_y, ego_yaw, ego_v : ego rear-bumper pose (m, rad) + speed.
%       start_point                  : 1x3 (unused — kept for inport wiring).
%       finish_point                 : 1x3 [x y *] T00 rear-bumper goal.
%       goal_yaw                     : T00 heading (rad; deg auto-converted).
%       occ_map                      : 200x200 occupancy grid from add_obstacle_.
%
%   Outputs
%       desired_ax       : longitudinal accel command -> AccelCtrl.DesiredAx
%                          (signed m/s^2; closed loop on target speed + gear).
%       steer_fl/fr      : front-wheel angle (rad), saturated.
%       path_x_dbg       : 300x1 path buffer (for monitoring).
%       path_y_dbg       : 300x1 path buffer.
%       path_len_dbg     : int32 — number of valid path samples.
%       selector_ctrl    : gear command -> DM.SelectorCtrl (+1 drive, -1 reverse).
%       vc_gas/brake     : legacy outputs, held at 0 (VC interface unused).
%
%   This file mirrors the chart script verbatim.  All helpers
%   (hybrid_astar_plan, stanley, pd_speed, rs_shot, compute_grid_heuristic,
%   compute_clearance, map_const) are inlined as local functions so the .slx
%   needs no external .m files.
%
%#codegen

MAX_PATH = int32(300);
% Static lot + fixed goal: plan ONCE and follow. The RRT* solve is heavy
% (~tens of seconds, interpreted), so the old 10000-step periodic replan
% caused a multi-second sim freeze every ~10 s. Disable periodic replan;
% we still replan on path-loss (len<2), goal change, and the one-shot
% goal-zone refinement (force_goal_replan within GOAL_REPLAN_RADIUS).
REPLAN_PERIOD = int32(2000000000);
GOAL_REPLAN_RADIUS = 6.5;

% path_dir stored as double (+1/-1) to avoid Stateflow int8 inference issues
persistent path_x path_y path_yaw path_dir path_len tick last_gx last_gy goal_zone_latched init
if isempty(init)
    path_x   = zeros(MAX_PATH, 1);
    path_y   = zeros(MAX_PATH, 1);
    path_yaw = zeros(MAX_PATH, 1);
    path_dir = ones(MAX_PATH, 1);
    path_len = int32(0);
    tick = int32(REPLAN_PERIOD + 1);
    last_gx = 1.0e9;
    last_gy = 1.0e9;
    goal_zone_latched = false;
    init = true;
end

u_unused = start_point(1) * 0;   %#ok<NASGU>

t00_x = finish_point(1);
t00_y = finish_point(2);
t00_yaw = goal_yaw;
if abs(ego_yaw) > 2.0 * pi; ego_yaw = ego_yaw * pi / 180.0; end
if abs(t00_yaw) > 2.0 * pi; t00_yaw = t00_yaw * pi / 180.0; end

d_goal_now = hypot(t00_x - ego_x, t00_y - ego_y);
yaw_goal_err = abs(wrap_pi(t00_yaw - ego_yaw));
goal_changed = abs(t00_x - last_gx) > 0.2 || abs(t00_y - last_gy) > 0.2;
if goal_changed
    goal_zone_latched = false;
end

if d_goal_now < 0.70 && yaw_goal_err < 0.35
    desired_ax = -1.5;          % firm decel to hold the car parked at the goal
    steer_fl = 0.0;
    steer_fr = 0.0;
    selector_ctrl = 1.0;
    vc_gas = 0.0;               % VC interface unused (held at 0)
    vc_brake = 0.0;
    if path_len < int32(2)
        path_x(:) = 0.0; path_y(:) = 0.0; path_yaw(:) = 0.0;
        path_dir(:) = 1.0;
        path_x(1) = ego_x; path_y(1) = ego_y; path_yaw(1) = ego_yaw;
        path_x(2) = t00_x; path_y(2) = t00_y; path_yaw(2) = t00_yaw;
        path_len = int32(2);
    end
    path_x_dbg = path_x;
    path_y_dbg = path_y;
    path_len_dbg = path_len;
    return;
end

force_goal_replan = false;
in_goal_replan_circle = d_goal_now < GOAL_REPLAN_RADIUS;
if in_goal_replan_circle && ~goal_zone_latched
    path_x(:) = 0.0;
    path_y(:) = 0.0;
    path_yaw(:) = 0.0;
    path_dir(:) = 1.0;
    path_len = int32(0);
    tick = int32(REPLAN_PERIOD + 1);
    goal_zone_latched = true;
    force_goal_replan = true;
elseif ~in_goal_replan_circle
    goal_zone_latched = false;
end

need_replan = false;
if path_len < int32(2); need_replan = true; end
if goal_changed; need_replan = true; end
if tick >= REPLAN_PERIOD; need_replan = true; end
if force_goal_replan; need_replan = true; end
if need_replan && d_goal_now < 2.5 && path_len >= int32(2) && ~force_goal_replan
    need_replan = false;
end

if need_replan
    [px, py, pyaw, pdir, plen] = rrt_ccrs_plan(ego_x, ego_y, ego_yaw, t00_x, t00_y, t00_yaw, uint8(occ_map));
    if plen >= int32(2)
        for i = int32(1):MAX_PATH
            path_x(i)   = px(i);
            path_y(i)   = py(i);
            path_yaw(i) = pyaw(i);
            path_dir(i) = double(pdir(i));
        end
        path_len = plen;
        tick = int32(0);
        last_gx = t00_x;
        last_gy = t00_y;
    elseif path_len < int32(2)
        path_x(:) = 0.0; path_y(:) = 0.0; path_yaw(:) = 0.0;
        path_dir(:) = 1.0;
        path_x(1) = ego_x; path_y(1) = ego_y; path_yaw(1) = ego_yaw;
        path_x(2) = ego_x; path_y(2) = ego_y; path_yaw(2) = ego_yaw;
        path_len = int32(2);
        tick = int32(0);
    end
end
tick = tick + int32(1);

stay_put = path_len <= int32(2) && hypot(path_x(2) - path_x(1), path_y(2) - path_y(1)) < 0.01;

% Item 11: fast on the open approach (compute_v_des), but keep the proven-safe
% SLOW endgame near the goal -- a faster run must NOT overshoot into the road.
if stay_put
    v_des = 0.0;
else
    v_des = compute_v_des(ego_x, ego_y, path_x, path_y, path_yaw, path_dir, path_len);
    d_goal = hypot(t00_x - ego_x, t00_y - ego_y);
    if d_goal <= 1.5
        if v_des > 0.25; v_des = 0.25; end
    elseif d_goal <= 6.0
        if v_des > 0.65; v_des = 0.65; end
    elseif d_goal <= 15.0
        if v_des > 0.85; v_des = 0.85; end
    end
end

% Convert path_dir (double) -> int8 buffer for stanley signature.
pdir_i8 = int8(zeros(MAX_PATH, 1));
for i = int32(1):MAX_PATH
    if path_dir(i) < 0
        pdir_i8(i) = int8(-1);
    else
        pdir_i8(i) = int8(1);
    end
end

[steer_cmd, dir_sign] = stanley(ego_x, ego_y, ego_yaw, ego_v, path_x, path_y, path_yaw, pdir_i8, path_len, t00_yaw);

dir_cmd = 1.0;
if dir_sign == int8(-1)
    dir_cmd = -1.0;
end
if stay_put
    dir_cmd = 1.0;
    v_des = 0.0;
end

[desired_ax, tire_angle, selector_ctrl, vc_gas, vc_brake] = ...
    control_with_shift_delay_local(ego_v, v_des, steer_cmd, dir_cmd);

steer_fl = tire_angle;
steer_fr = tire_angle;

% Genuine RRT* + Reeds-Shepp(CCRS) plan -> Stanley lateral + PD longitudinal.
% Override removed: desired_ax / steer / gear above come from the planner +
% Stanley follower (lines computing steer_cmd, v_des, dir_cmd). If the car
% does not start, scope path_len_dbg: < 2 means rrt_ccrs_plan found NO path
% (stay_put -> v_des = 0); >= 2 means a path exists and it is a control issue.

path_x_dbg = path_x;
path_y_dbg = path_y;
path_len_dbg = path_len;
end

%% =====================================================================
%% Local helpers - RRT* + CCRS planner (replaces Hybrid A*)
%% =====================================================================

function [px, py, pyaw, pdir, plen] = rrt_ccrs_plan(sx, sy, syaw, gx, gy, gyaw, occ_map)
%RRT_CCRS_PLAN  Reverse-parking planner: RRT* over SE(2) with Reeds-Shepp
%   connections (CCRS endgame), for the Day4_5 parking scenario.
%
%   All poses are REAR-BUMPER center (m, rad); body extends +yaw from local-x 0
%   to EGO_L. Output path_x/y/yaw/dir/len matches the existing cache+controller
%   contract (dir +1 forward / -1 reverse).
%
%   v1: RRT* finds the route; an analytic Reeds-Shepp steering function
%   (CSC + CCC, forward AND reverse with correct per-segment directions)
%   connects nodes and shoots the goal. Collision uses a half-width-inflated
%   plan map + ego centerline sampling (fast); the exact 2D footprint is
%   re-verified offline. CC clothoid smoothing is layered on top (v2).
%
%#codegen

MAX_PATH  = int32(300);
MAX_NODES = int32(8000);
N_ITER    = int32(20000);
EXTRA_AFTER_GOAL = int32(600);    % post-goal refine iters. 600 trims the one-time
                                  % interpreted RRT* solve from ~22s to ~14s while
                                  % still reaching the goal exactly (footprint-verified).
GOAL_BIAS  = 0.15;
STAGE_BIAS = 0.20;
W_YAW      = 1.8;                % heading weight in the node distance metric
RN_GAMMA   = 30.0;              % RRT* rewire-radius gamma
RN_MIN     = 4.0;
RN_MAX     = 6.0;               % small radius: a 40 m lot fills fast, keeps rewire cost down

% ---- normalise inputs ----------------------------------------------------
if abs(syaw) > 2.0*pi; syaw = syaw * pi/180.0; end
if abs(gyaw) > 2.0*pi; gyaw = gyaw * pi/180.0; end

c = map_const_local();
R_RS = c.WHEELBASE / tan(0.50);   % min turning radius (~5.13 m)
DS   = 0.30;                      % RS path sample spacing

% ---- collision model: half-width-inflated binary plan map ----------------
% Inflating the obstacle map by (ego half width + lateral margin) lets us
% collision-check the ego by sampling only its longitudinal centerline.
% half-width + lateral margin, plus a discretisation cushion (~0.40 m) that
% absorbs the obstacle-raster edge loss (~0.25 m) and the chamfer distance
% overestimate (~0.10 m), so the centerline model never lets a footprint
% corner clip an obstacle (verified against the exact 2D footprint offline).
inflate_r = c.EGO_W * 0.5 + c.EGO_WIDTH_SAFETY_MARGIN + 0.30;
plan_map  = build_plan_map(uint8(occ_map), inflate_r);

% ---- start back-out corridor --------------------------------------------
% The start is parked nose-to-boundary: the ego footprint pokes outside the
% lot wall (e.g. start 5.5,-36.5 heading west -> front at x=0.35 < lot edge 4).
% Without this, every edge from the root collides near the start and the tree
% never grows. Free the swept reverse back-out corridor so RRT* can launch
% the (physically valid) first reverse maneuver. The corridor is local to the
% start and verified clear of real obstacles.
plan_map = carve_start_corridor(plan_map, sx, sy, syaw, c);

% ---- sampling bounds from the free region --------------------------------
[xlo, xhi, ylo, yhi] = free_bounds(plan_map, c);

% ---- domain-knowledge stage points (off BOTH ends of the stall) ----------
% The car always approaches from the slot's OPEN (aisle) side, but which side
% that is in goal-heading frame depends on whether the final move is forward
% (nose-in) or reverse (back-in). Stage on BOTH +gyaw and -gyaw sides so the
% aisle side is always guided, regardless of parking direction.
stage_d  = 3.0;
stgx_a = gx + stage_d * cos(gyaw);
stgy_a = gy + stage_d * sin(gyaw);
stgx_b = gx - stage_d * cos(gyaw);
stgy_b = gy - stage_d * sin(gyaw);

% ---- RRT* node storage ---------------------------------------------------
nx   = zeros(MAX_NODES, 1);
ny   = zeros(MAX_NODES, 1);
nyaw = zeros(MAX_NODES, 1);
ncost = zeros(MAX_NODES, 1);
npar = zeros(MAX_NODES, 1, 'int32');

n = int32(1);
nx(1) = sx; ny(1) = sy; nyaw(1) = syaw; ncost(1) = 0.0; npar(1) = int32(0);

best_goal = int32(0);
best_goal_cost = 1.0e18;
goal_found_iter = int32(0);

for it = int32(1):N_ITER
    if best_goal > int32(0) && (it - goal_found_iter) > EXTRA_AFTER_GOAL
        break;
    end
    if n >= MAX_NODES
        break;
    end

    % ---- sample a target pose --------------------------------------------
    rsel = rand();
    if rsel < GOAL_BIAS
        qx = gx; qy = gy; qyaw = gyaw;
    elseif rsel < GOAL_BIAS + STAGE_BIAS
        if rand() < 0.5
            bx = stgx_a; by = stgy_a;
        else
            bx = stgx_b; by = stgy_b;
        end
        qx = bx + (rand() - 0.5) * 6.0;
        qy = by + (rand() - 0.5) * 6.0;
        qyaw = gyaw + (rand() - 0.5) * 0.8;
    else
        qx = xlo + (xhi - xlo) * rand();
        qy = ylo + (yhi - ylo) * rand();
        qyaw = -pi + 2.0*pi*rand();
    end

    % ---- nearest existing node -------------------------------------------
    inn = int32(1);
    best_d = 1.0e18;
    for j = int32(1):n
        d = node_dist(nx(j), ny(j), nyaw(j), qx, qy, qyaw, W_YAW);
        if d < best_d
            best_d = d;
            inn = j;
        end
    end

    % ---- connect nearest -> q (RS). reject if infeasible/colliding -------
    [okc, lenc] = rs_len(nx(inn), ny(inn), nyaw(inn), qx, qy, qyaw, plan_map, R_RS, DS, MAX_PATH);
    if ~okc
        continue;
    end

    % ---- choose best parent within the rewire radius ---------------------
    rn = near_radius(n, RN_GAMMA, RN_MIN, RN_MAX);
    bp = inn;
    bc = ncost(inn) + lenc;
    for j = int32(1):n
        if j == inn; continue; end
        dj = hypot(nx(j) - qx, ny(j) - qy);
        if dj > rn; continue; end
        [okj, lenj] = rs_len(nx(j), ny(j), nyaw(j), qx, qy, qyaw, plan_map, R_RS, DS, MAX_PATH);
        if okj && (ncost(j) + lenj) < bc
            bp = j;
            bc = ncost(j) + lenj;
        end
    end

    % ---- add the new node ------------------------------------------------
    n = n + int32(1);
    nx(n) = qx; ny(n) = qy; nyaw(n) = qyaw;
    ncost(n) = bc; npar(n) = bp;
    newidx = n;

    % ---- rewire neighbours through the new node --------------------------
    for j = int32(1):(n - int32(1))
        dj = hypot(nx(j) - qx, ny(j) - qy);
        if dj > rn; continue; end
        [okr, lenr] = rs_len(qx, qy, qyaw, nx(j), ny(j), nyaw(j), plan_map, R_RS, DS, MAX_PATH);
        if okr && (bc + lenr) < ncost(j)
            npar(j) = newidx;
            ncost(j) = bc + lenr;
        end
    end

    % ---- try to connect the new node to the goal -------------------------
    [okg, leng] = rs_len(qx, qy, qyaw, gx, gy, gyaw, plan_map, R_RS, DS, MAX_PATH);
    if okg && (bc + leng) < best_goal_cost
        best_goal_cost = bc + leng;
        best_goal = newidx;
        if goal_found_iter == int32(0)
            goal_found_iter = it;
        end
    end
end

% ---- reconstruct the best start->goal path -------------------------------
[px, py, pyaw, pdir, plen] = reconstruct_path( ...
    nx, ny, nyaw, npar, n, best_goal, gx, gy, gyaw, plan_map, R_RS, DS, MAX_PATH);
end

%% =====================================================================
%% RRT* support
%% =====================================================================

function d = node_dist(x, y, yaw, qx, qy, qyaw, w_yaw)
%#codegen
d = hypot(x - qx, y - qy) + w_yaw * abs(wrap_pi(yaw - qyaw));
end

function rn = near_radius(n, gamma, rmin, rmax)
%#codegen
nn = double(n);
if nn < 2.0
    rn = rmax;
    return;
end
rn = gamma * (log(nn) / nn)^(1.0/3.0);
if rn < rmin; rn = rmin; end
if rn > rmax; rn = rmax; end
end

function [ok, total_len] = rs_len(sx, sy, syaw, gx, gy, gyaw, occ_map, R, ds, MAX_PATH)
%#codegen
% Reeds-Shepp connect + collision check; returns success and path length.
[px, py, ~, ~, plen, ok] = rs_shot(sx, sy, syaw, gx, gy, gyaw, occ_map, R, ds);
total_len = 1.0e18;
if ~ok || plen < int32(2)
    ok = false;
    return;
end
L = 0.0;
for i = int32(1):(plen - int32(1))
    L = L + hypot(px(i+1) - px(i), py(i+1) - py(i));
end
total_len = L;
end

function [px, py, pyaw, pdir, plen] = reconstruct_path( ...
    nx, ny, nyaw, npar, n, best_goal, gx, gy, gyaw, occ_map, R, ds, MAX_PATH)
%#codegen
BIG = int32(4000);
bx = zeros(1, BIG); by = zeros(1, BIG); byaw = zeros(1, BIG);
bdir = zeros(1, BIG, 'int8');
blen = int32(0);

px   = zeros(1, MAX_PATH);
py   = zeros(1, MAX_PATH);
pyaw = zeros(1, MAX_PATH);
pdir = zeros(1, MAX_PATH, 'int8');
plen = int32(0);

if best_goal < int32(1)
    % fallback: no goal connection found -> 2-point stay-put at start
    px(1) = nx(1); py(1) = ny(1); pyaw(1) = nyaw(1); pdir(1) = int8(1);
    px(2) = nx(1); py(2) = ny(1); pyaw(2) = nyaw(1); pdir(2) = int8(1);
    plen = int32(2);
    return;
end

% walk parents: build chain [start ... best_goal]
chain = zeros(1, MAX_PATH, 'int32');
clen = int32(0);
node = best_goal;
while node > int32(0) && clen < MAX_PATH
    clen = clen + int32(1);
    chain(clen) = node;
    node = npar(node);
end
% reverse chain into start->goal order
half = clen / int32(2);
for i = int32(1):half
    tmp = chain(i);
    chain(i) = chain(clen - i + int32(1));
    chain(clen - i + int32(1)) = tmp;
end

% concatenate RS sub-paths between consecutive chain nodes
for k = int32(1):(clen - int32(1))
    a = chain(k); b = chain(k + int32(1));
    [sx2, sy2, syaw2, sdir2, slen2, sok] = ...
        rs_shot(nx(a), ny(a), nyaw(a), nx(b), ny(b), nyaw(b), occ_map, R, ds);
    if ~sok || slen2 < int32(2); continue; end
    [bx, by, byaw, bdir, blen] = append_seg(bx, by, byaw, bdir, blen, ...
        sx2, sy2, syaw2, sdir2, slen2, BIG, (blen == int32(0)));
end

% final shot from the last chain node to the goal
last = chain(clen);
[fx, fy, fyaw, fdir, flen, fok] = ...
    rs_shot(nx(last), ny(last), nyaw(last), gx, gy, gyaw, occ_map, R, ds);
if fok && flen >= int32(2)
    [bx, by, byaw, bdir, blen] = append_seg(bx, by, byaw, bdir, blen, ...
        fx, fy, fyaw, fdir, flen, BIG, (blen == int32(0)));
end

if blen < int32(2)
    px(1) = nx(1); py(1) = ny(1); pyaw(1) = nyaw(1); pdir(1) = int8(1);
    px(2) = gx;    py(2) = gy;    pyaw(2) = gyaw;    pdir(2) = int8(1);
    plen = int32(2);
    return;
end

% resample to <= MAX_PATH, preserving direction-change (cusp) vertices
[px, py, pyaw, pdir, plen] = resample_path(bx, by, byaw, bdir, blen, MAX_PATH);

% snap the final point exactly onto the goal pose
if plen >= int32(1)
    px(plen) = gx; py(plen) = gy; pyaw(plen) = gyaw;
end
end

function [bx, by, byaw, bdir, blen] = append_seg(bx, by, byaw, bdir, blen, ...
        sx, sy, syaw, sdir, slen, BIG, is_first)
%#codegen
% append a sub-path; skip its first sample when joining (shared endpoint)
start_i = int32(2);
if is_first; start_i = int32(1); end
for i = start_i:slen
    if blen >= BIG; break; end
    blen = blen + int32(1);
    bx(blen) = sx(i); by(blen) = sy(i); byaw(blen) = syaw(i); bdir(blen) = sdir(i);
end
end

function [px, py, pyaw, pdir, plen] = resample_path(bx, by, byaw, bdir, blen, MAX_PATH)
%#codegen
px   = zeros(1, MAX_PATH);
py   = zeros(1, MAX_PATH);
pyaw = zeros(1, MAX_PATH);
pdir = zeros(1, MAX_PATH, 'int8');

if blen <= MAX_PATH
    for i = int32(1):blen
        px(i) = bx(i); py(i) = by(i); pyaw(i) = byaw(i); pdir(i) = bdir(i);
    end
    plen = blen;
    return;
end

% keep first, last, every direction-change vertex, and enforce a min spacing
% so the total fits in MAX_PATH.
total_len = 0.0;
for i = int32(1):(blen - int32(1))
    total_len = total_len + hypot(bx(i+1) - bx(i), by(i+1) - by(i));
end
min_ds = total_len / (double(MAX_PATH) - 2.0);

plen = int32(1);
px(1) = bx(1); py(1) = by(1); pyaw(1) = byaw(1); pdir(1) = bdir(1);
acc = 0.0;
for i = int32(2):(blen - int32(1))
    acc = acc + hypot(bx(i) - bx(i-1), by(i) - by(i-1));
    cusp = (bdir(i) ~= bdir(i-1));
    if (acc >= min_ds || cusp) && plen < (MAX_PATH - int32(1))
        plen = plen + int32(1);
        px(plen) = bx(i); py(plen) = by(i); pyaw(plen) = byaw(i); pdir(plen) = bdir(i);
        acc = 0.0;
    end
end
plen = plen + int32(1);
px(plen) = bx(blen); py(plen) = by(blen); pyaw(plen) = byaw(blen); pdir(plen) = bdir(blen);
end

%% =====================================================================
%% Collision model: inflated plan map + free-region bounds
%% =====================================================================

function plan_map = build_plan_map(occ_map, inflate_r)
%#codegen
% binary map inflated by inflate_r metres (via clearance distance transform)
clr = compute_clearance(uint8(occ_map));
c = map_const_local();
plan_map = uint8(occ_map);
for r = int32(1):c.N
    for cc = int32(1):c.N
        if clr(r, cc) < single(inflate_r)
            plan_map(r, cc) = uint8(1);
        end
    end
end
end

function plan_map = carve_start_corridor(plan_map, sx, sy, syaw, c)
%#codegen
% Free the cells the ego sweeps while reversing straight out of its parked
% (nose-to-wall) start pose. Reverse travel is opposite the heading.
rdx = -cos(syaw); rdy = -sin(syaw);
cyaw = cos(syaw); syaw_ = sin(syaw);
D_BACKOUT = c.EGO_L + 3.5;
half_w  = c.EGO_W * 0.5 + c.EGO_WIDTH_SAFETY_MARGIN + 0.6;   % slightly wide for curved launch
x_back  = -c.EGO_REAR_SAFETY_MARGIN;
x_front =  c.EGO_L + c.EGO_FRONT_SAFETY_MARGIN;
nd = int32(ceil(D_BACKOUT / 0.20)) + int32(1);
nl = int32(ceil((x_front - x_back) / 0.20)) + int32(1);
nw = int32(ceil((2.0 * half_w) / 0.20)) + int32(1);
for id = int32(1):nd
    d = D_BACKOUT * (double(id) - 1.0) / (double(nd) - 1.0);
    bxc = sx + d * rdx;
    byc = sy + d * rdy;
    for il = int32(1):nl
        sl = x_back + (x_front - x_back) * (double(il) - 1.0) / (double(nl) - 1.0);
        for iw = int32(1):nw
            lt = -half_w + (2.0 * half_w) * (double(iw) - 1.0) / (double(nw) - 1.0);
            wx = bxc + sl * cyaw - lt * syaw_;
            wy = byc + sl * syaw_ + lt * cyaw;
            if wx < c.X_MIN || wx > c.X_MAX || wy < c.Y_MIN || wy > c.Y_MAX
                continue;
            end
            col = int32(floor((wx - c.X_MIN) / c.RES)) + int32(1);
            row = int32(floor((c.Y_MAX - wy) / c.RES)) + int32(1);
            if row >= 1 && row <= c.N && col >= 1 && col <= c.N
                plan_map(row, col) = uint8(0);
            end
        end
    end
end
end

function [xlo, xhi, ylo, yhi] = free_bounds(plan_map, c)
%#codegen
rmin = c.N; rmax = int32(1); cmin = c.N; cmax = int32(1);
found = false;
for r = int32(1):c.N
    for cc = int32(1):c.N
        if plan_map(r, cc) == uint8(0)
            found = true;
            if r < rmin; rmin = r; end
            if r > rmax; rmax = r; end
            if cc < cmin; cmin = cc; end
            if cc > cmax; cmax = cc; end
        end
    end
end
if ~found
    xlo = c.X_MIN; xhi = c.X_MAX; ylo = c.Y_MIN; yhi = c.Y_MAX;
    return;
end
% cell centre -> world (col->x increasing, row->y decreasing)
xlo = c.X_MIN + (double(cmin) - 0.5) * c.RES;
xhi = c.X_MIN + (double(cmax) - 0.5) * c.RES;
yhi = c.Y_MAX - (double(rmin) - 0.5) * c.RES;
ylo = c.Y_MAX - (double(rmax) - 0.5) * c.RES;
end

%% =====================================================================
%% Reeds-Shepp steering (analytic CSC + CCC, forward AND reverse)
%% =====================================================================
%   Adapted from the canonical Reeds-Shepp word formulas (Reeds & Shepp 1990;
%   symmetry organisation as in OMPL / PythonRobotics). Each candidate word is
%   up to 3 segments with SIGNED lengths (unit-curvature units); the sign of a
%   segment length is its travel direction (+ forward / - reverse), so reverse
%   maneuvers come out with correct gear labels. The shortest valid word that
%   is collision-free is returned.

function [px, py, pyaw, pdir, plen, ok] = rs_shot(sx, sy, syaw, gx, gy, gyaw, occ_map, R, ds)
%#codegen
MAX_PATH = int32(300);
px   = zeros(1, MAX_PATH);
py   = zeros(1, MAX_PATH);
pyaw = zeros(1, MAX_PATH);
pdir = zeros(1, MAX_PATH, 'int8');
plen = int32(0);
ok   = false;

% transform goal into the start-local, unit-curvature frame
maxc = 1.0 / R;
dx = gx - sx; dy = gy - sy;
cs = cos(syaw); sn = sin(syaw);
x   = ( cs*dx + sn*dy) * maxc;
y   = (-sn*dx + cs*dy) * maxc;
phi = wrap_pi(gyaw - syaw);

[seglen, segmode, found] = rs_best_word(x, y, phi);
if ~found
    return;
end

[px, py, pyaw, pdir, plen] = rs_interp(seglen, segmode, sx, sy, syaw, R, ds, MAX_PATH);
if plen < int32(2)
    plen = int32(0);
    return;
end
px(plen) = gx; py(plen) = gy; pyaw(plen) = gyaw;   % snap exact goal

for i = int32(1):plen-int32(1)
    if is_collision_segment(px(i), py(i), pyaw(i), px(i+1), py(i+1), pyaw(i+1), occ_map)
        plen = int32(0);
        return;
    end
end
ok = true;
end

function [best_len, best_mode, found] = rs_best_word(x, y, phi)
%#codegen
% Try every CSC (LSL,LSR) and CCC (LRL) word under the 4 symmetry transforms.
best_len  = zeros(1, 3);
best_mode = zeros(1, 3);     % 1=L, 2=S, 3=R
best_cost = 1.0e18;
found = false;

for base_id = int32(1):int32(3)
    for tf = 0:1
        for rf = 0:1
            [len3, mode3, cost, fl] = rs_try(base_id, tf == 1, rf == 1, x, y, phi);
            if fl && cost < best_cost
                best_cost = cost;
                best_len  = len3;
                best_mode = mode3;
                found = true;
            end
        end
    end
end
end

function [len3, mode3, cost, flag] = rs_try(base_id, tf, rf, x, y, phi)
%#codegen
len3 = zeros(1, 3); mode3 = zeros(1, 3); cost = 1.0e18; flag = false;

% symmetry transform of the query (timeflip / reflect)
x_in = x; if tf; x_in = -x; end
y_in = y; if rf; y_in = -y; end
phi_in = phi; if xor(tf, rf); phi_in = -phi; end

if base_id == int32(1)
    [t, u, v, fl] = rs_LSL(x_in, y_in, phi_in);  base_mode = [1 2 1];
elseif base_id == int32(2)
    [t, u, v, fl] = rs_LSR(x_in, y_in, phi_in);  base_mode = [1 2 3];
else
    [t, u, v, fl] = rs_LRL(x_in, y_in, phi_in);  base_mode = [1 3 1];
end
if ~fl
    return;
end

if tf                       % timeflip negates segment lengths (reverse travel)
    t = -t; u = -u; v = -v;
end
len3 = [t, u, v];

mode3 = base_mode;
if rf                       % reflect swaps L<->R
    for i = 1:3
        if mode3(i) == 1
            mode3(i) = 3;
        elseif mode3(i) == 3
            mode3(i) = 1;
        end
    end
end

cost = abs(t) + abs(u) + abs(v);
flag = true;
end

function [t, u, v, flag] = rs_LSL(x, y, phi)
%#codegen
[u, t] = rs_polar(x - sin(phi), y - 1.0 + cos(phi));
v = wrap_pi(phi - t);
flag = (t >= -1.0e-9) && (v >= -1.0e-9);
end

function [t, u, v, flag] = rs_LSR(x, y, phi)
%#codegen
t = 0.0; u = 0.0; v = 0.0; flag = false;
[r1, t1] = rs_polar(x + sin(phi), y - 1.0 - cos(phi));
u1sq = r1 * r1;
if u1sq >= 4.0
    u = sqrt(u1sq - 4.0);
    theta = atan2(2.0, u);
    t = wrap_pi(t1 + theta);
    v = wrap_pi(t - phi);
    flag = (t >= -1.0e-9) && (v >= -1.0e-9);
end
end

function [t, u, v, flag] = rs_LRL(x, y, phi)
%#codegen
t = 0.0; u = 0.0; v = 0.0; flag = false;
[r1, t1] = rs_polar(x - sin(phi), y - 1.0 + cos(phi));
if r1 <= 4.0
    u = -2.0 * asin(0.25 * r1);            % middle (R) segment, u <= 0
    t = wrap_pi(t1 + 0.5 * u + pi);
    v = wrap_pi(phi - t + u);
    flag = (t >= -1.0e-9) && (u <= 1.0e-9);
end
end

function [r, th] = rs_polar(a, b)
%#codegen
r = hypot(a, b);
th = atan2(b, a);
end

function [px, py, pyaw, pdir, plen] = rs_interp(seglen, segmode, sx, sy, syaw, R, ds, MAX_PATH)
%#codegen
% Sample a Reeds-Shepp word (segment lengths in unit-curvature units) into a
% pose sequence. Per-segment direction = sign(seglen); heading rate set by mode.
px   = zeros(1, MAX_PATH);
py   = zeros(1, MAX_PATH);
pyaw = zeros(1, MAX_PATH);
pdir = zeros(1, MAX_PATH, 'int8');

idx = int32(1);
px(1) = sx; py(1) = sy; pyaw(1) = syaw;
first_dir = int8(1);
for s = 1:3
    if abs(seglen(s)) > 1.0e-9
        if seglen(s) < 0.0; first_dir = int8(-1); else; first_dir = int8(1); end
        break;
    end
end
pdir(1) = first_dir;

ox = sx; oy = sy; oyaw = syaw;
for s = 1:3
    realL = seglen(s) * R;            % signed real length
    if abs(realL) < 1.0e-6
        continue;
    end
    m = segmode(s);
    if realL < 0.0; dseg = int8(-1); else; dseg = int8(1); end
    nstep = int32(ceil(abs(realL) / ds));
    if nstep < int32(1); nstep = int32(1); end

    x0 = ox; y0 = oy; yaw0 = oyaw;
    cx = 0.0; cy = 0.0;
    if m == 1            % L : centre to the left
        cx = x0 - R * sin(yaw0);
        cy = y0 + R * cos(yaw0);
    elseif m == 3        % R : centre to the right
        cx = x0 + R * sin(yaw0);
        cy = y0 - R * cos(yaw0);
    end

    xk = x0; yk = y0; yawk = yaw0;
    for k = int32(1):nstep
        a = realL * (double(k) / double(nstep));   % cumulative signed arc
        if m == 2                       % S
            xk = x0 + a * cos(yaw0);
            yk = y0 + a * sin(yaw0);
            yawk = yaw0;
        elseif m == 1                   % L
            yawk = yaw0 + a / R;
            xk = cx + R * sin(yawk);
            yk = cy - R * cos(yawk);
        else                            % R
            yawk = yaw0 - a / R;
            xk = cx - R * sin(yawk);
            yk = cy + R * cos(yawk);
        end
        if idx >= MAX_PATH
            break;
        end
        idx = idx + int32(1);
        px(idx) = xk; py(idx) = yk; pyaw(idx) = wrap_pi(yawk); pdir(idx) = dseg;
    end
    ox = xk; oy = yk; oyaw = yawk;
end
plen = idx;
end

%% =====================================================================
%% Collision check (fast: centerline vs inflated plan map)
%% =====================================================================

function col = is_collision_segment(x1, y1, yaw1, x2, y2, yaw2, occ_map)
%#codegen
col = false;
for k = int32(0):int32(3)
    t = double(k) / 3.0;
    x = x1 + t * (x2 - x1);
    y = y1 + t * (y2 - y1);
    yaw = wrap_pi(yaw1 + t * wrap_pi(yaw2 - yaw1));
    if is_occupied_pose(x, y, yaw, occ_map)
        col = true;
        return;
    end
end
end

function occ = is_occupied_pose(x, y, yaw, occ_map)
%#codegen
% occ_map is the half-width-inflated plan map, so the ego footprint reduces to
% its longitudinal centerline from x_back to x_front.
c = map_const_local();
c_yaw = cos(yaw);
s_yaw = sin(yaw);
occ = false;

x_back  = -c.EGO_REAR_SAFETY_MARGIN;
x_front =  c.EGO_L + c.EGO_FRONT_SAFETY_MARGIN;
n_long = int32(ceil((x_front - x_back) / c.FOOTPRINT_SAMPLE_DS)) + int32(1);
if n_long < int32(2); n_long = int32(2); end

for il = int32(1):n_long
    s_lng = x_back + (x_front - x_back) * (double(il) - 1.0) / (double(n_long) - 1.0);
    qx = x + s_lng * c_yaw;
    qy = y + s_lng * s_yaw;
    if is_occupied_point(qx, qy, occ_map)
        occ = true;
        return;
    end
end
end

function occ = is_occupied_point(x, y, occ_map)
%#codegen
c = map_const_local();
occ = false;
if x < c.X_MIN || x > c.X_MAX || y < c.Y_MIN || y > c.Y_MAX
    occ = true;
    return;
end
col = int32(floor((x - c.X_MIN) / c.RES)) + int32(1);
row = int32(floor((c.Y_MAX - y) / c.RES)) + int32(1);
if row < 1 || row > c.N || col < 1 || col > c.N
    occ = true;
    return;
end
if occ_map(row, col) > 0
    occ = true;
end
end

function clear_map = compute_clearance(occ_map)
%#codegen
c = map_const_local();
N = double(c.N);
res = c.RES;
N_int = int32(N);

INF = single(1.0e9);
clear_map = INF * ones(N, N, 'single');

MAX_Q = int32(8 * N * N);
qr = zeros(MAX_Q, 1, 'int32');
qc = zeros(MAX_Q, 1, 'int32');
qhead = int32(1);
qtail = int32(1);

for r = int32(1):N_int
    for cc = int32(1):N_int
        if occ_map(r, cc) > 0
            clear_map(r, cc) = single(0);
            qr(qtail) = r;
            qc(qtail) = cc;
            qtail = qtail + int32(1);
        end
    end
end

DR = int32([-1 -1 -1  0  0  1  1  1]);
DC = int32([-1  0  1 -1  1 -1  0  1]);
SQ2 = single(sqrt(2.0));
COSTS = single([SQ2 1 SQ2 1 1 SQ2 1 SQ2]) * single(res);

while qhead < qtail
    r = qr(qhead);
    cc = qc(qhead);
    qhead = qhead + int32(1);
    d_here = clear_map(r, cc);
    for k = int32(1):int32(8)
        nr = r + DR(k);
        nc = cc + DC(k);
        if nr < 1 || nr > N_int || nc < 1 || nc > N_int
            continue;
        end
        new_d = d_here + COSTS(k);
        if new_d < clear_map(nr, nc)
            clear_map(nr, nc) = new_d;
            if qtail <= MAX_Q
                qr(qtail) = nr;
                qc(qtail) = nc;
                qtail = qtail + int32(1);
            end
        end
    end
end
end

%% =====================================================================
%% Angle / constant helpers
%% =====================================================================

function a = wrap_pi(a)
%#codegen
while a > pi;  a = a - 2.0 * pi; end
while a < -pi; a = a + 2.0 * pi; end
end

function c = map_const_local()
%#codegen
c.N        = int32(200);
c.RES      = 0.5;
c.X_MIN    = 0.0;
c.X_MAX    = 100.0;
c.Y_MIN    = -100.0;
c.Y_MAX    = 0.0;
c.EGO_W    = 1.9;
c.EGO_L    = 4.7;
c.WHEELBASE = 2.8;
c.REAR_AXLE_FROM_REAR_BUMPER = 0.95;
c.EGO_WIDTH_SAFETY_MARGIN = 0.35;
c.EGO_FRONT_SAFETY_MARGIN = 0.45;
c.EGO_REAR_SAFETY_MARGIN  = 0.50;
c.FOOTPRINT_SAMPLE_DS     = 0.35;
end


%% =====================================================================
%% Local helpers — Stanley lateral controller
%% =====================================================================

function [steer_cmd, dir_sign] = stanley(ego_x, ego_y, ego_yaw, ego_v, ...
                                          path_x, path_y, path_yaw, path_dir, ...
                                          path_len, goal_yaw)
%#codegen
c = map_const_local();
MAX_PATH    = int32(300);
MAX_STEER   = 0.5;
K_E         = 1.5;
V_SOFT      = 1.0;
END_RADIUS  = 3.0;

steer_cmd = 0.0;
dir_sign = int8(1);
if path_len < int32(2)
    return;
end

plen = int32(min(int32(path_len), MAX_PATH));

nearest_idx = int32(1);
best_d2 = 1.0e18;
for i = int32(1):plen
    dx = path_x(i) - ego_x;
    dy = path_y(i) - ego_y;
    d2 = dx*dx + dy*dy;
    if d2 < best_d2
        best_d2 = d2;
        nearest_idx = i;
    end
end

have_path_yaw = false;
if numel(path_yaw) >= double(plen)
    if any(abs(path_yaw(1:min(plen, int32(20)))) > 1.0e-6)
        have_path_yaw = true;
    end
end

if have_path_yaw
    path_heading = path_yaw(nearest_idx);
else
    if nearest_idx < plen
        path_heading = atan2(path_y(nearest_idx+1) - path_y(nearest_idx), ...
                             path_x(nearest_idx+1) - path_x(nearest_idx));
    elseif nearest_idx > 1
        path_heading = atan2(path_y(nearest_idx) - path_y(nearest_idx-1), ...
                             path_x(nearest_idx) - path_x(nearest_idx-1));
    else
        path_heading = goal_yaw;
    end
end

end_dist = 0.0;
prev_x = path_x(nearest_idx);
prev_y = path_y(nearest_idx);
for i = nearest_idx+int32(1):plen
    end_dist = end_dist + hypot(path_x(i) - prev_x, path_y(i) - prev_y);
    prev_x = path_x(i);
    prev_y = path_y(i);
end

if end_dist < END_RADIUS
    alpha = 1.0 - end_dist / END_RADIUS;
    if alpha < 0.0; alpha = 0.0; end
    if alpha > 1.0; alpha = 1.0; end
    diff = wrap_pi(goal_yaw - path_heading);
    path_heading = wrap_pi(path_heading + alpha * diff);
end

if numel(path_dir) >= double(nearest_idx)
    dir_idx = nearest_idx;
    if nearest_idx < plen
        dir_idx = nearest_idx + int32(1);
    end
    dir_sign = int8(sign_default(path_dir(dir_idx), int8(1)));
end

heading_err = wrap_pi(path_heading - ego_yaw);

dx = ego_x - path_x(nearest_idx);
dy = ego_y - path_y(nearest_idx);
c_ph = cos(path_heading);
s_ph = sin(path_heading);
cross_track =  s_ph * dx - c_ph * dy;

cross_track_eff = cross_track;
if dir_sign == int8(-1)
    cross_track_eff = -cross_track;
end
% Curvature feedforward: the steady-state steering that holds the path's own
% curvature (bicycle: tan(delta) = L*kappa).  Without it Stanley is pure
% feedback and lags on curves (corner-cutting / overshoot, worse at speed).
% kappa = d(path_yaw)/ds averaged over a short lookahead.  Added BEFORE the
% reverse flip below, so the existing negation gives the correct reverse sign
% (since d(psi)/ds = dir * tan(delta)/L).
steer_ff = 0.0;
if have_path_yaw
    L_FF  = 3.0;
    k_sum = 0.0;
    k_cnt = int32(0);
    acc   = 0.0;
    j = nearest_idx;
    while j < plen && acc < L_FF
        seg = hypot(path_x(j+1) - path_x(j), path_y(j+1) - path_y(j));
        if seg > 1.0e-6
            dpsi  = wrap_pi(path_yaw(j+1) - path_yaw(j));
            k_sum = k_sum + dpsi / seg;
            k_cnt = k_cnt + int32(1);
            acc   = acc + seg;
        end
        j = j + int32(1);
    end
    if k_cnt > int32(0)
        steer_ff = atan(c.WHEELBASE * (k_sum / double(k_cnt)));
    end
end

steer_cmd = heading_err + atan2(K_E * cross_track_eff, V_SOFT + abs(ego_v)) + steer_ff;
if dir_sign == int8(-1)
    steer_cmd = -steer_cmd;
end

if steer_cmd > MAX_STEER
    steer_cmd = MAX_STEER;
elseif steer_cmd < -MAX_STEER
    steer_cmd = -MAX_STEER;
end

u_unused = c.WHEELBASE * 0.0;       %#ok<NASGU>
end

%% =====================================================================
%% Local helpers — PID longitudinal controller
%% =====================================================================

function [desired_ax, tire_angle, vc_selector, vc_gas, vc_brake] = ...
    control_with_shift_delay_local(ego_v, target_v, steer_cmd, dir_cmd)
%#codegen
% Acceleration-interface parking control (interface unification, per feedback):
%   longitudinal command -> AccelCtrl.DesiredAx (signed accel m/s^2, out1)
%   gear command         -> DM.SelectorCtrl  (+1 drive / -1 reverse, out7)
%   VC.Gas / VC.Brake are NOT used (held at 0).
%
% Reverse stability + anti-surge: we (a) commit the gear via DM.SelectorCtrl,
% (b) run a PD outer loop on the SPEED-MAGNITUDE target (target_v - |ego_v|)
% whose output is the desired acceleration, and (c) feed that to
% AccelCtrl.DesiredAx as a GEAR-AWARE request: AccelCtrl.DesiredAx is read in
% the engaged gear's TRAVEL direction, so a POSITIVE value accelerates the car
% in its gear direction (forward in D, backward in R) and a NEGATIVE value
% brakes. We do NOT flip the sign by gear (a sign-flip once fed a "brake"
% command in reverse, so the car never moved); the gear commit alone removes
% the v=0 pedal-sign flip (manual 6.5.8).
%
% 급발진(surge) ROOT CAUSE + fix: the longitudinal chain is a CASCADE -- our
% command feeds CarMaker's inner AccelCtrl PI (p=0.001, i=1.0), which is itself
% an integrator. An OUTER integral here = two integrators in series -> windup ->
% surge. So the outer loop is PD ONLY: P tracks the target speed, D-on-
% MEASUREMENT (-KD*d|v|/dt, no derivative kick) damps it, and the inner PI owns
% the steady state (commanding desired_ax=0 makes it hold speed against drag).
% Reverse uses lower gains/caps + a soft rate-limited launch (~AX_START).
%   desired_ax = KP*e - KD*d|v|/dt ,  e = v_mag_tgt - |v|
% Gear changes still wait at standstill before DM.SelectorCtrl is switched.
persistent active_selector switch_count spd_prev d_lpf ax_cmd

ego_v    = ego_v(1);
target_v = abs(target_v(1));     % magnitude; gear handled by DM.SelectorCtrl
steer_cmd = steer_cmd(1);
dir_cmd  = dir_cmd(1);

if isempty(active_selector)
    active_selector = 1.0;
    switch_count = int32(0);
    spd_prev = 0.0;
    d_lpf    = 0.0;
    ax_cmd   = 0.0;
end

tire_angle = steer_cmd;
speed_abs  = abs(ego_v);

% ---- requested gear from the path's local direction --------------------
requested_selector = 1.0;
if dir_cmd < 0.0
    requested_selector = -1.0;
end

% ---- standstill-gated gear shift --------------------------------------
shifting = false;
if requested_selector ~= active_selector
    shifting    = true;
    tire_angle  = 0.0;           % straighten wheels while the gear engages
    if speed_abs < 0.12
        switch_count = switch_count + int32(1);
        if switch_count > int32(15)   % 기어전환 대기 50->15틱(0.5->0.15s): cusp 정지간격 단축
            active_selector = requested_selector;
            switch_count = int32(0);
            shifting = false;
        end
    else
        switch_count = int32(0);
    end
else
    switch_count = int32(0);
end

vc_selector = active_selector;   % -> DM.SelectorCtrl
vc_gas      = 0.0;               % VC interface intentionally unused
vc_brake    = 0.0;

% ---- gear-dependent gentleness (REVERSE anti-surge) -------------------
% Root cause of the reverse 급가속: this is a CASCADE of two integrators. Our
% PID outputs desired_ax, which feeds CarMaker's INNER AccelCtrl PI
% (p=0.001, i=1.0 ~ a near-pure integrator) that winds VC.Gas up until the
% ACTUAL accel reaches our command. With a large command (forward used
% AX_MAX=0.40 toward a 0.60 m/s cruise) that inner integrator pushes gas hard;
% forward the powertrain keeps up, but in REVERSE the EV regen/creep lags, so
% the inner integrator over-accumulates and dumps it at once -> surge. The sign
% is correct (positive = accelerate in gear dir, confirmed empirically); the
% fix is MAGNITUDE/DYNAMICS. Forward parking already works, so we ONLY soften
% the committed-reverse leg: low creep target, low accel cap, slow rise, soft
% P/I. This keeps the inner PI in a small, well-behaved regime (no runaway).
% 검증 레시피(Parking_simple)와 동일한 캡/게인. 후진은 살살(낮은 속도상한).
is_rev = active_selector < 0.0;
if is_rev
    V_TGT_MAX = 0.30;           % 후진 속도상한(살살)
else
    V_TGT_MAX = 0.70;           % 전진 속도상한
end
KP_LON     = 0.8;               % 속도 P 게인
AX_MAX     = 0.6;               % 가속 캡
AX_MIN     = -1.5;              % 브레이크 캡
AX_SLEW_UP = 0.05;              % a_desr 상승률 제한(가스 완만 -> AccelCtrl PI surge 완화)

% ---- 종방향: 검증 레시피 (속도 P + slew-rate-limit, Parking_simple 검증본) ----
% 속도 P(미분 없음) + a_desr 상승률 제한: 가스(상승)만 완만히 → AccelCtrl 내부 PI 가
% 과반응(surge)하지 않게 / 브레이크(하강)는 자유 → 빨리 멈춤. (기존 PD+launch 대체)
v_mag_tgt = target_v;
if v_mag_tgt > V_TGT_MAX
    v_mag_tgt = V_TGT_MAX;
end
if shifting || target_v < 0.05
    v_mag_tgt = 0.0;             % 기어전환 중/목표 도달 -> 정지
end

e = v_mag_tgt - speed_abs;
desired_ax = KP_LON * e;         % 속도 P (미분 없음)
if desired_ax > AX_MAX; desired_ax = AX_MAX; end
if desired_ax < AX_MIN; desired_ax = AX_MIN; end
if v_mag_tgt == 0.0 && speed_abs < 0.05
    desired_ax = 0.0;            % 완전 정지 유지
end
% 상승률 제한(가스만 완만). ax_cmd 를 이전 명령(prev_ax)으로 재사용.
if desired_ax > ax_cmd + AX_SLEW_UP
    desired_ax = ax_cmd + AX_SLEW_UP;
end
ax_cmd = desired_ax;
end

function desired_ax = pd_speed(v_des, v_ego)
%#codegen
DT = 0.01;
KP = 1.2;
KD = 0.25;
AX_MIN = -3.0;
AX_MAX = 0.8;
ALPHA  = 0.6;

persistent e_prev d_lpf init
if isempty(init)
    e_prev = 0.0;
    d_lpf  = 0.0;
    init   = true;
end

e   = v_des - abs(v_ego);
e_d = (e - e_prev) / DT;
d_lpf = ALPHA * d_lpf + (1.0 - ALPHA) * e_d;
e_prev = e;

desired_ax = KP * e + KD * d_lpf;

if desired_ax > AX_MAX
    desired_ax = AX_MAX;
elseif desired_ax < AX_MIN
    desired_ax = AX_MIN;
end
end

%% =====================================================================
%% Local helpers — path-geometry speed profile (item 11)
%% =====================================================================

function v = compute_v_des(ego_x, ego_y, path_x, path_y, path_yaw, path_dir, path_len)
%#codegen
% Path-geometry speed target: fast on straight forward segments, slowed by
% (a) remaining stopping distance, (b) upcoming curvature, (c) an imminent
% gear change.  Returns a non-negative magnitude; sign/gear handled by caller.
    V_MAX_FWD = 1.2;     % forward top speed in the parking lot              [tunable]
    V_MAX_REV = 0.30;    % reverse top speed (살살: windup 최소화, 검증본과 일치) [tunable]
A_BRAKE   = 2.5;     % usable decel for stopping distance (< |AX_MIN|=3) [tunable]
A_LAT     = 2.0;     % comfortable lateral accel -> corner speed cap     [tunable]
    V_MIN     = 0.25;    % creep floor while still en route
LOOK_M    = 8.0;     % curvature / gear-change lookahead distance (m)
KAPPA_MIN = 1.0e-3;
STOP_S    = 0.5;     % within this remaining arc-length -> stop

N = int32(numel(path_x));
plen = path_len;
if plen > N; plen = N; end
if plen < int32(2)
    v = 0.0;
    return;
end

% (1) nearest path index to ego
nidx = int32(1);
best = 1.0e18;
for i = int32(1):plen
    dx = path_x(i) - ego_x;
    dy = path_y(i) - ego_y;
    d2 = dx*dx + dy*dy;
    if d2 < best
        best = d2;
        nidx = i;
    end
end

% (2) remaining arc-length from nidx to the goal
s_rem = 0.0;
for i = nidx:(plen - int32(1))
    s_rem = s_rem + hypot(path_x(i+1) - path_x(i), path_y(i+1) - path_y(i));
end

% (3) lookahead: max curvature + distance to the next gear (direction) change
kap_max = 0.0;
s_acc   = 0.0;
dir_idx = nidx;
if nidx < plen
    dir_idx = nidx + int32(1);
end
dir_here = path_dir(dir_idx);
dir_change_s = 1.0e18;
i = nidx;
while i < plen && s_acc < LOOK_M
    seg = hypot(path_x(i+1) - path_x(i), path_y(i+1) - path_y(i));
    if seg > 1.0e-6
        dyaw = wrap_pi(path_yaw(i+1) - path_yaw(i));
        kap = abs(dyaw) / seg;
        if kap > kap_max
            kap_max = kap;
        end
    end
    seg_dir = path_dir(i+1);
    if seg_dir ~= dir_here && dir_change_s > 1.0e17
        dir_change_s = s_acc + seg;
    end
    s_acc = s_acc + seg;
    i = i + int32(1);
end

% (4) combine speed limits (take the minimum)
v = V_MAX_FWD;
if dir_here < 0.0
    v = V_MAX_REV;
end
v_arc   = sqrt(2.0 * A_BRAKE * max(s_rem, 0.0));         % can still stop within s_rem
v_curv  = sqrt(A_LAT / max(kap_max, KAPPA_MIN));         % corner speed cap
v_shift = sqrt(2.0 * A_BRAKE * max(dir_change_s, 0.0));  % near-0 by the gear change
if v_arc   < v; v = v_arc;   end
if v_curv  < v; v = v_curv;  end
if v_shift < v; v = v_shift; end

% creep floor unless essentially at the goal
if s_rem > STOP_S
    if v < V_MIN
        v = V_MIN;
    end
else
    v = 0.0;
end
end

function s = sign_default(v, fallback)
%#codegen
if v > 0
    s = int8(1);
elseif v < 0
    s = int8(-1);
else
    s = fallback;
end
end
