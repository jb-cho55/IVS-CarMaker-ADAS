function [desired_ax, steer_fl, steer_fr, path_x_dbg, path_y_dbg, path_len_dbg, selector_ctrl, vc_gas, vc_brake] = Parking(ego_x, ego_y, ego_yaw, ego_v, start_point, finish_point, goal_yaw, occ_map)
%PARKING  Self-contained planner + controllers for the Day4_5_Scenario_1.slx
%   "MATLAB Function" (Parking) block.
%
%   Hybrid A* (with Reeds-Shepp analytic-shot endgame) -> Stanley + PD.
%
%   Inputs
%       ego_x, ego_y, ego_yaw, ego_v : ego rear-bumper pose (m, rad) + speed.
%       start_point                  : 1x3 (unused — kept for inport wiring).
%       finish_point                 : 1x3 [x y *] T00 rear-bumper goal.
%       goal_yaw                     : T00 heading (rad; deg auto-converted).
%       occ_map                      : 200x200 occupancy grid from add_obstacle_.
%
%   Outputs
%       desired_ax       : compatibility output; held at 0 for EV6 VC control.
%       steer_fl/fr      : front-wheel angle (rad), saturated.
%       path_x_dbg       : 300x1 path buffer (for monitoring).
%       path_y_dbg       : 300x1 path buffer.
%       path_len_dbg     : int32 — number of valid path samples.
%       selector_ctrl    : VC.SelectCtrl (+1 drive, -1 reverse).
%       vc_gas/brake     : direct EV6 pedal commands.
%
%   This file mirrors the chart script verbatim.  All helpers
%   (hybrid_astar_plan, stanley, pd_speed, rs_shot, compute_grid_heuristic,
%   compute_clearance, map_const) are inlined as local functions so the .slx
%   needs no external .m files.
%
%#codegen

MAX_PATH = int32(300);
REPLAN_PERIOD = int32(10000);
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
    desired_ax = 0.0;
    steer_fl = 0.0;
    steer_fr = 0.0;
    selector_ctrl = 1.0;
    vc_gas = 0.0;
    vc_brake = 0.55;
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
    [px, py, pyaw, pdir, plen] = two_stage_parking_plan(ego_x, ego_y, ego_yaw, t00_x, t00_y, t00_yaw, uint8(occ_map));
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

path_x_dbg = path_x;
path_y_dbg = path_y;
path_len_dbg = path_len;
end

%% =====================================================================
%% Local helpers — Hybrid A* planner
%% =====================================================================

function [path_x, path_y, path_yaw, path_dir, path_len] = two_stage_parking_plan(sx, sy, syaw, gx, gy, gyaw, occ_map)
%#codegen
MAX_PATH = int32(300);
path_x   = zeros(1, MAX_PATH);
path_y   = zeros(1, MAX_PATH);
path_yaw = zeros(1, MAX_PATH);
path_dir = zeros(1, MAX_PATH, 'int8');
path_len = int32(0);

c = map_const_local();
plan_map = inflate_map_for_planning(uint8(occ_map), c.PLANNER_EXTRA_MARGIN);

R_RS = c.WHEELBASE / tan(0.50);
DS_RS = 0.45;
if hypot(sx - gx, sy - gy) < 18.0
    [rs_direct_x, rs_direct_y, rs_direct_yaw, rs_direct_dir, rs_direct_len, rs_direct_ok] = ...
        rs_shot(sx, sy, syaw, gx, gy, gyaw, plan_map, R_RS, DS_RS);
    if rs_direct_ok && rs_direct_len >= int32(2)
        path_x = rs_direct_x;
        path_y = rs_direct_y;
        path_yaw = rs_direct_yaw;
        path_dir = rs_direct_dir;
        path_len = rs_direct_len;
        return;
    end
end

if hypot(sx - gx, sy - gy) < 2.75
    [path_x, path_y, path_yaw, path_dir, path_len] = ...
        local_finish_plan(sx, sy, syaw, gx, gy, gyaw, plan_map);
    return;
end

stage_dist = 4.5;
stage_x = gx + stage_dist * cos(gyaw);
stage_y = gy + stage_dist * sin(gyaw);
stage_yaw = gyaw;

if hypot(sx - stage_x, sy - stage_y) < 3.0
    [path_x, path_y, path_yaw, path_dir, path_len] = ...
        hybrid_astar_plan(sx, sy, syaw, gx, gy, gyaw, plan_map, false);
    return;
end

fx = zeros(1, MAX_PATH);
fy = zeros(1, MAX_PATH);
fyaw = zeros(1, MAX_PATH);
fdir = zeros(1, MAX_PATH, 'int8');
flen = int32(0);

approach_dist = hypot(sx - stage_x, sy - stage_y);
if approach_dist > 16.0
    [gx_path, gy_path, glen] = grid_approach_plan(sx, sy, stage_x, stage_y, plan_map);
    if glen >= int32(2)
        flen = glen;
        for gi = int32(1):glen
            fx(gi) = gx_path(gi);
            fy(gi) = gy_path(gi);
            fdir(gi) = int8(1);
        end
        for gi = int32(1):glen
            if gi < glen
                fyaw(gi) = atan2(fy(gi+1) - fy(gi), fx(gi+1) - fx(gi));
            elseif gi > int32(1)
                fyaw(gi) = fyaw(gi-1);
            else
                fyaw(gi) = syaw;
            end
        end
    end
end

if flen < int32(2)
    [fx, fy, fyaw, fdir, flen, fok] = ...
        rs_shot(sx, sy, syaw, stage_x, stage_y, stage_yaw, plan_map, R_RS, DS_RS);
    if ~fok || flen < int32(2)
        [fx, fy, fyaw, fdir, flen] = ...
            hybrid_astar_plan(sx, sy, syaw, stage_x, stage_y, stage_yaw, plan_map, true);
        if flen < int32(2)
            [gx_path, gy_path, glen] = grid_approach_plan(sx, sy, stage_x, stage_y, plan_map);
            if glen < int32(2)
                [path_x, path_y, path_yaw, path_dir, path_len] = hold_position_path(sx, sy, syaw);
                return;
            end
            fx(:) = 0.0; fy(:) = 0.0; fyaw(:) = 0.0; fdir(:) = int8(1);
            flen = glen;
            for gi = int32(1):glen
                fx(gi) = gx_path(gi);
                fy(gi) = gy_path(gi);
                fdir(gi) = int8(1);
            end
            for gi = int32(1):glen
                if gi < glen
                    fyaw(gi) = atan2(fy(gi+1) - fy(gi), fx(gi+1) - fx(gi));
                elseif gi > int32(1)
                    fyaw(gi) = fyaw(gi-1);
                else
                    fyaw(gi) = syaw;
                end
            end
        end
    end
end

for i = int32(1):flen
    if path_len >= MAX_PATH
        break;
    end
    path_len = path_len + int32(1);
    path_x(path_len) = fx(i);
    path_y(path_len) = fy(i);
    path_yaw(path_len) = fyaw(i);
    path_dir(path_len) = fdir(i);
end

[path_x, path_y, path_yaw, path_dir, path_len] = ...
    smooth_forward_bspline_constrained(path_x, path_y, path_yaw, path_dir, path_len, plan_map);

[rx, ry, ryaw, rdir, rlen] = hybrid_astar_plan(stage_x, stage_y, stage_yaw, gx, gy, gyaw, plan_map, false);
if rlen >= int32(2)
    for i = int32(2):rlen
        if path_len >= MAX_PATH
            break;
        end
        path_len = path_len + int32(1);
        path_x(path_len) = rx(i);
        path_y(path_len) = ry(i);
        path_yaw(path_len) = ryaw(i);
        path_dir(path_len) = rdir(i);
    end
else
    return;
end
end

function [path_x, path_y, path_yaw, path_dir, path_len] = hold_position_path(sx, sy, syaw)
%#codegen
MAX_PATH = int32(300);
path_x   = zeros(1, MAX_PATH);
path_y   = zeros(1, MAX_PATH);
path_yaw = zeros(1, MAX_PATH);
path_dir = zeros(1, MAX_PATH, 'int8');
path_len = int32(2);
path_x(1) = sx; path_y(1) = sy; path_yaw(1) = syaw; path_dir(1) = int8(1);
path_x(2) = sx; path_y(2) = sy; path_yaw(2) = syaw; path_dir(2) = int8(1);
end

function [path_x, path_y, path_yaw, path_dir, path_len] = ...
    local_finish_plan(sx, sy, syaw, gx, gy, gyaw, occ_map)
%#codegen
MAX_PATH = int32(300);
path_x   = zeros(1, MAX_PATH);
path_y   = zeros(1, MAX_PATH);
path_yaw = zeros(1, MAX_PATH);
path_dir = zeros(1, MAX_PATH, 'int8');
path_len = int32(0);

dx = gx - sx;
dy = gy - sy;
dist = hypot(dx, dy);

move_yaw = syaw;
if dist > 1.0e-6
    move_yaw = atan2(dy, dx);
end

dir0 = int8(1);
if cos(wrap_pi(move_yaw - syaw)) < 0.0
    dir0 = int8(-1);
end

nseg = int32(ceil(dist / 0.25)) + int32(1);
if nseg < int32(2); nseg = int32(2); end
if nseg > int32(24); nseg = int32(24); end

tx = zeros(1, MAX_PATH);
ty = zeros(1, MAX_PATH);
tyaw = zeros(1, MAX_PATH);
tdir = zeros(1, MAX_PATH, 'int8');

for i = int32(1):nseg
    t = double(i - int32(1)) / double(nseg - int32(1));
    tx(i) = sx + t * dx;
    ty(i) = sy + t * dy;
    tyaw(i) = wrap_pi(syaw + t * wrap_pi(gyaw - syaw));
    tdir(i) = dir0;
end

if is_candidate_path_safe(tx, ty, tyaw, nseg, occ_map)
    path_x = tx;
    path_y = ty;
    path_yaw = tyaw;
    path_dir = tdir;
    path_len = nseg;
    return;
end

STEERS = [-0.50, -0.25, 0.0, 0.25, 0.50];
DIRS = int8([dir0, -dir0]);
best_cost = 1.0e9;
best_x = zeros(1, MAX_PATH);
best_y = zeros(1, MAX_PATH);
best_yaw = zeros(1, MAX_PATH);
best_dir = zeros(1, MAX_PATH, 'int8');
best_len = int32(0);

for di = int32(1):int32(2)
    dir_a = DIRS(di);
    for si = int32(1):int32(5)
        steer = STEERS(si);
        ax = zeros(1, MAX_PATH);
        ay = zeros(1, MAX_PATH);
        ayaw = zeros(1, MAX_PATH);
        adir = zeros(1, MAX_PATH, 'int8');
        ax(1) = sx; ay(1) = sy; ayaw(1) = syaw; adir(1) = dir_a;
        alen = int32(1);
        ok = true;
        travel = max(1.0, min(3.5, dist + 0.8));
        steps = int32(ceil(travel / 0.30));
        if steps > int32(18); steps = int32(18); end
        for k = int32(1):steps
            if alen >= MAX_PATH - int32(1)
                break;
            end
            [nx_a, ny_a, nyaw_a] = bicycle_step(ax(alen), ay(alen), ayaw(alen), steer, 0.30 * double(dir_a), 2.8);
            if is_collision_segment(ax(alen), ay(alen), ayaw(alen), nx_a, ny_a, nyaw_a, occ_map)
                ok = false;
                break;
            end
            alen = alen + int32(1);
            ax(alen) = nx_a; ay(alen) = ny_a; ayaw(alen) = nyaw_a; adir(alen) = dir_a;
        end
        if ~ok
            continue;
        end
        d_end = hypot(gx - ax(alen), gy - ay(alen));
        if d_end > 1.20
            continue;
        end
        if is_collision_segment(ax(alen), ay(alen), ayaw(alen), gx, gy, gyaw, occ_map)
            continue;
        end
        if alen < MAX_PATH
            alen = alen + int32(1);
            ax(alen) = gx; ay(alen) = gy; ayaw(alen) = gyaw; adir(alen) = dir_a;
        end
        cost = d_end + 1.5 * abs(wrap_pi(gyaw - ayaw(max(alen - int32(1), int32(1))))) + 0.2 * abs(steer);
        if cost < best_cost
            best_cost = cost;
            best_x = ax; best_y = ay; best_yaw = ayaw; best_dir = adir; best_len = alen;
        end
    end
end

if best_len >= int32(2)
    path_x = best_x;
    path_y = best_y;
    path_yaw = best_yaw;
    path_dir = best_dir;
    path_len = best_len;
    return;
end

% If the inflated local finish is not safe, do not launch a heavy search at
% the goal. Hold position; this is preferable to lagging into obstacles.
path_len = int32(2);
path_x(1) = sx; path_y(1) = sy; path_yaw(1) = syaw; path_dir(1) = int8(1);
path_x(2) = sx; path_y(2) = sy; path_yaw(2) = syaw; path_dir(2) = int8(1);
end

function [px, py, plen] = grid_approach_plan(sx, sy, gx, gy, occ_map)
%#codegen
MAX_PATH = int32(300);
px = zeros(1, MAX_PATH);
py = zeros(1, MAX_PATH);
plen = int32(0);

c = map_const_local();
search_map = inflate_map_for_planning(uint8(occ_map), c.EGO_W * 0.5 + c.PATH_WIDTH_MARGIN);
h_grid = compute_grid_heuristic(search_map, gx, gy);

[row, col] = world_to_cell_local(sx, sy);
[row, col] = nearest_free_cell(search_map, row, col);

h_here = h_grid(row, col);
if h_here >= single(1.0e8)
    return;
end

plen = int32(1);
px(plen) = sx;
py(plen) = sy;

DR = int32([-1 -1 -1  0  0  1  1  1]);
DC = int32([-1  0  1 -1  1 -1  0  1]);

for step = int32(1):int32(220)
    best_r = row;
    best_c = col;
    best_h = h_grid(row, col);

    for k = int32(1):int32(8)
        nr = row + DR(k);
        nc = col + DC(k);
        if nr < int32(1) || nr > c.N || nc < int32(1) || nc > c.N
            continue;
        end
        if search_map(nr, nc) > 0
            continue;
        end
        nh = h_grid(nr, nc);
        if nh < best_h
            best_h = nh;
            best_r = nr;
            best_c = nc;
        end
    end

    if best_r == row && best_c == col
        break;
    end

    row = best_r;
    col = best_c;

    if mod(double(step), 2.0) == 0.0
        if plen < MAX_PATH
            plen = plen + int32(1);
            [wx, wy] = cell_to_world_local(row, col);
            px(plen) = wx;
            py(plen) = wy;
        end
    end

    if best_h < single(0.75)
        break;
    end
end

if plen < MAX_PATH
    plen = plen + int32(1);
    px(plen) = gx;
    py(plen) = gy;
end
end

function [row, col] = world_to_cell_local(x, y)
%#codegen
c = map_const_local();
col = int32(floor((x - c.X_MIN) / c.RES)) + int32(1);
row = int32(floor((c.Y_MAX - y) / c.RES)) + int32(1);
if col < int32(1); col = int32(1); end
if col > c.N; col = c.N; end
if row < int32(1); row = int32(1); end
if row > c.N; row = c.N; end
end

function [x, y] = cell_to_world_local(row, col)
%#codegen
c = map_const_local();
x = c.X_MIN + (double(col) - 0.5) * c.RES;
y = c.Y_MAX - (double(row) - 0.5) * c.RES;
end

function [row_o, col_o] = nearest_free_cell(occ_map, row, col)
%#codegen
c = map_const_local();
row_o = row;
col_o = col;
if occ_map(row, col) == 0
    return;
end
for rad = int32(1):int32(20)
    for dr = -rad:rad
        for dc = -rad:rad
            if abs(double(dr)) ~= double(rad) && abs(double(dc)) ~= double(rad)
                continue;
            end
            nr = row + dr;
            nc = col + dc;
            if nr < int32(1) || nr > c.N || nc < int32(1) || nc > c.N
                continue;
            end
            if occ_map(nr, nc) == 0
                row_o = nr;
                col_o = nc;
                return;
            end
        end
    end
end
end

function [path_x, path_y, path_yaw, path_dir, path_len] = hybrid_astar_plan(sx, sy, syaw, gx, gy, gyaw, occ_map, allow_mixed)
%#codegen
MAX_NODES = int32(18000);
MAX_PATH  = int32(300);
N_STEER   = int32(5);

WHEELBASE   = 2.8;
STEP_DIST   = 0.8;
STEERS = [-0.50, -0.25, 0.0, 0.25, 0.50];
ACTION_PENALTY = 0.35;
GEAR_CHANGE_PENALTY = 8.0;
REVERSE_PENALTY = 0.20;
POS_RES = 0.5;
YAW_RES = pi / 12.0;
shared = map_const_local();
CLEAR_MAX = shared.CLEAR_MAX;
W_CLEAR   = shared.W_CLEAR;
BOX_L     = shared.PARK_BOX_L;
BOX_W     = shared.PARK_BOX_W;
BOX_TOL   = shared.PARK_TOL;
EGO_L     = shared.EGO_L;
EGO_W     = shared.EGO_W;
W_HEUR   = 1.3;
W_YAW    = 1.0;

plan_map = uint8(occ_map);
h_grid    = compute_grid_heuristic(plan_map, gx, gy);
clear_map = compute_clearance(plan_map);

path_x   = zeros(1, MAX_PATH);
path_y   = zeros(1, MAX_PATH);
path_yaw = zeros(1, MAX_PATH);
path_dir = zeros(1, MAX_PATH, 'int8');
path_len = int32(0);

nx  = zeros(MAX_NODES, 1);
ny  = zeros(MAX_NODES, 1);
nyaw = zeros(MAX_NODES, 1);
ng  = zeros(MAX_NODES, 1);
nf  = zeros(MAX_NODES, 1);
nparent = zeros(MAX_NODES, 1, 'int32');
ndir    = zeros(MAX_NODES, 1, 'int8');
nclosed = false(MAX_NODES, 1);

% Indexed binary min-heap over the open set, keyed by (nf, node index).
% heap(1..heap_size) holds open-node indices; heap_pos(node) is that node's
% slot (0 = not in heap).  Replaces the old O(node_count) linear min-f scan
% with O(log N) push/pop/decrease-key -- the last O(N^2) in the planner.
heap      = zeros(MAX_NODES, 1, 'int32');
heap_pos  = zeros(MAX_NODES, 1, 'int32');
heap_size = int32(0);

kx = zeros(MAX_NODES, 1, 'int32');
ky = zeros(MAX_NODES, 1, 'int32');
kw = zeros(MAX_NODES, 1, 'int32');

node_count = int32(1);
sy_w = wrap_pi(syaw);
nx(1)   = sx;
ny(1)   = sy;
nyaw(1) = sy_w;
ng(1)   = 0.0;
h0 = lookup_h(sx, sy, h_grid) + W_YAW * abs(angle_diff(gyaw, sy_w));
nf(1)   = W_HEUR * h0;
nparent(1) = int32(0);
ndir(1)    = int8(1);
if ~allow_mixed
    ndir(1) = int8(-1);
end
[heap, heap_pos, heap_size] = heap_push(heap, heap_pos, heap_size, int32(1), nf);
kx(1) = int32(round(sx / POS_RES));
ky(1) = int32(round(sy / POS_RES));
kw(1) = int32(round(wrap_2pi(sy_w) / YAW_RES));

% O(1) duplicate-state detection via a dense 3D lookup table indexed by the
% discrete key (kx, ky, kw) -> node index (0 = empty).  Replaces the old
% O(node_count) linear scan (which made hard plans O(N^2) ~ minutes).  Sized
% to cover the on-grid range plus the start-bubble margin; any key outside
% falls back to a (rare) linear scan so correctness is unconditional.
% idx mapping: ix = kx + KX_OFF, iy = ky + KY_OFF, iw = kw + 1.
KX_N = int32(221); KY_N = int32(211); KW_N = int32(26);
KX_OFF = int32(16); KY_OFF = int32(206);
lut = zeros(KX_N, KY_N, KW_N, 'int32');
ix1 = kx(1) + KX_OFF; iy1 = ky(1) + KY_OFF; iw1 = kw(1) + int32(1);
if ix1 >= 1 && ix1 <= KX_N && iy1 >= 1 && iy1 <= KY_N && iw1 >= 1 && iw1 <= KW_N
    lut(ix1, iy1, iw1) = int32(1);
end

goal_idx = int32(0);
rs_attached = false;
rs_px   = zeros(1, MAX_PATH);
rs_py   = zeros(1, MAX_PATH);
rs_pyaw = zeros(1, MAX_PATH);
rs_pdir = zeros(1, MAX_PATH, 'int8');
rs_plen = int32(0);
start_pose_occupied = is_occupied_pose(sx, sy, sy_w, plan_map);

iter_limit = MAX_NODES;
sg_dist = hypot(gx - sx, gy - sy);
if sg_dist < 4.0
    iter_limit = int32(2500);
elseif sg_dist < 8.0
    iter_limit = int32(5000);
elseif sg_dist < 14.0
    iter_limit = int32(9000);
end

for iter = int32(1):iter_limit
    if heap_size == int32(0)
        break;
    end
    cur = heap(1);
    [heap, heap_pos, heap_size] = heap_remove_top(heap, heap_pos, heap_size, nf);
    nclosed(cur) = true;

    cx = nx(cur); cy = ny(cur); cyaw = nyaw(cur);

    if pose_in_goal_box(cx, cy, cyaw, gx, gy, gyaw, ...
                        BOX_L, BOX_W, BOX_TOL, EGO_L, EGO_W)
        goal_idx = cur;
        break;
    end

    d_to_goal = hypot(gx - cx, gy - cy);
    try_rs = false;
    if iter == int32(1)
        try_rs = true;
    elseif mod(iter, int32(20)) == int32(0)
        try_rs = true;
    end
    if d_to_goal < 12.0 && try_rs
        R = WHEELBASE / tan(0.5);
        [rs_px, rs_py, rs_pyaw, rs_pdir, rs_plen, rs_ok] = ...
            rs_shot(cx, cy, cyaw, gx, gy, gyaw, plan_map, R, STEP_DIST);
        if rs_ok && rs_plen >= int32(2)
            goal_idx = cur;
            rs_attached = true;
            break;
        end
    end

    n_dir = int32(1);
    dirs = int8([-1, 1]);
    if allow_mixed
        n_dir = int32(2);
        dirs(1) = int8(1);
        dirs(2) = int8(-1);
    end

    for di = int32(1):n_dir
        dir_a = dirs(di);
        ds = STEP_DIST * double(dir_a);
        for a = int32(1):N_STEER
            delta = STEERS(a);

            [nx_n, ny_n, nyaw_n] = bicycle_step(cx, cy, cyaw, delta, ds, WHEELBASE);

            start_escape = start_pose_occupied && hypot(cx - sx, cy - sy) < 1.4;
            if ~start_escape && is_collision_segment(cx, cy, cyaw, nx_n, ny_n, nyaw_n, plan_map)
                continue;
            end

            kxn = int32(round(nx_n / POS_RES));
            kyn = int32(round(ny_n / POS_RES));
            kwn = int32(round(wrap_2pi(nyaw_n) / YAW_RES));

            ixn = kxn + KX_OFF; iyn = kyn + KY_OFF; iwn = kwn + int32(1);
            in_lut = (ixn >= 1 && ixn <= KX_N && iyn >= 1 && iyn <= KY_N && ...
                      iwn >= 1 && iwn <= KW_N);
            dup_idx = int32(0);
            if in_lut
                dup_idx = lut(ixn, iyn, iwn);
            else
                for j = int32(1):node_count   % fallback (key outside LUT bounds)
                    if kx(j) == kxn && ky(j) == kyn && kw(j) == kwn
                        dup_idx = j;
                        break;
                    end
                end
            end

            step_cost = STEP_DIST * (1.0 + 0.2 * abs(delta)) + ACTION_PENALTY;
            if dir_a < int8(0)
                step_cost = step_cost + REVERSE_PENALTY;
            end
            if ndir(cur) ~= dir_a
                step_cost = step_cost + GEAR_CHANGE_PENALTY;
            end
            clr = lookup_h(nx_n, ny_n, clear_map);
            if clr < CLEAR_MAX
                step_cost = step_cost + W_CLEAR * (CLEAR_MAX - clr);
            end
            new_g = ng(cur) + step_cost;
            h_grid_n = lookup_h(nx_n, ny_n, h_grid);
            if h_grid_n >= 1.0e9
                h_grid_n = hypot(gx - nx_n, gy - ny_n);
            end
            h = h_grid_n + W_YAW * abs(angle_diff(gyaw, nyaw_n));
            new_f = new_g + W_HEUR * h;

            if dup_idx ~= int32(0)
                if nclosed(dup_idx); continue; end
                if new_g >= ng(dup_idx); continue; end
                nx(dup_idx)  = nx_n;
                ny(dup_idx)  = ny_n;
                nyaw(dup_idx) = nyaw_n;
                ng(dup_idx)  = new_g;
                nf(dup_idx)  = new_f;
                nparent(dup_idx) = cur;
                ndir(dup_idx)    = dir_a;
                % dup is open (closed nodes were rejected above) -> already in the
                % heap.  A better-g relaxation can RAISE f (the overwritten pose may
                % have a worse heuristic), so re-sift in whichever direction needed.
                [heap, heap_pos] = heap_resift(heap, heap_pos, heap_size, heap_pos(dup_idx), nf);
            else
                if node_count >= MAX_NODES
                    break;
                end
                node_count = node_count + int32(1);
                nx(node_count)   = nx_n;
                ny(node_count)   = ny_n;
                nyaw(node_count) = nyaw_n;
                ng(node_count)   = new_g;
                nf(node_count)   = new_f;
                nparent(node_count) = cur;
                ndir(node_count)    = dir_a;
                kx(node_count) = kxn;
                ky(node_count) = kyn;
                kw(node_count) = kwn;
                if in_lut
                    lut(ixn, iyn, iwn) = node_count;
                end
                [heap, heap_pos, heap_size] = heap_push(heap, heap_pos, heap_size, node_count, nf);
            end
        end
    end
end

if goal_idx == int32(0)
    return;
end

tmp_x   = zeros(MAX_PATH, 1);
tmp_y   = zeros(MAX_PATH, 1);
tmp_yaw = zeros(MAX_PATH, 1);
tmp_dir = zeros(MAX_PATH, 1, 'int8');
cnt = int32(0);
idx = goal_idx;
while idx > int32(0) && cnt < MAX_PATH
    cnt = cnt + int32(1);
    tmp_x(cnt)   = nx(idx);
    tmp_y(cnt)   = ny(idx);
    tmp_yaw(cnt) = nyaw(idx);
    tmp_dir(cnt) = ndir(idx);
    idx = nparent(idx);
end

for k = int32(1):cnt
    src = cnt - k + int32(1);
    path_x(k)   = tmp_x(src);
    path_y(k)   = tmp_y(src);
    path_yaw(k) = tmp_yaw(src);
    path_dir(k) = tmp_dir(src);
end
path_len = cnt;

if rs_attached && rs_plen >= int32(2)
    for k = int32(2):rs_plen
        if path_len >= MAX_PATH
            break;
        end
        path_len = path_len + int32(1);
        path_x(path_len)   = rs_px(k);
        path_y(path_len)   = rs_py(k);
        path_yaw(path_len) = rs_pyaw(k);
        path_dir(path_len) = rs_pdir(k);
    end
end

[path_x, path_y, path_yaw, path_dir, path_len] = ...
    smooth_bspline_constrained(path_x, path_y, path_yaw, path_dir, path_len, plan_map);
end

%% =====================================================================
%% Local helpers — constrained B-spline smoothing
%% =====================================================================

function [out_x, out_y, out_yaw, out_dir, out_len] = ...
    smooth_bspline_constrained(in_x, in_y, in_yaw, in_dir, in_len, occ_map)
%#codegen
MAX_PATH = int32(300);
out_x = in_x;
out_y = in_y;
out_yaw = in_yaw;
out_dir = in_dir;
out_len = in_len;

plen = in_len;
if plen < int32(5)
    return;
end
if plen > MAX_PATH
    plen = MAX_PATH;
end

all_reverse = true;
for i = int32(1):plen
    if in_dir(i) > int8(0)
        all_reverse = false;
        break;
    end
end
if ~all_reverse
    return;
end

ctrl_x = in_x;
ctrl_y = in_y;
alpha = 0.35;

for iter = int32(1):int32(6)
    for i = int32(2):plen-int32(1)
        cand_x = ctrl_x(i) + alpha * (0.5 * (ctrl_x(i-1) + ctrl_x(i+1)) - ctrl_x(i));
        cand_y = ctrl_y(i) + alpha * (0.5 * (ctrl_y(i-1) + ctrl_y(i+1)) - ctrl_y(i));

        yaw_a = reverse_centerline_yaw(ctrl_x(i-1), ctrl_y(i-1), cand_x, cand_y, ctrl_x(i+1), ctrl_y(i+1));
        yaw_b = reverse_centerline_yaw(cand_x, cand_y, ctrl_x(i+1), ctrl_y(i+1), ...
                                       ctrl_x(min(i+2, plen)), ctrl_y(min(i+2, plen)));
        yaw_prev = reverse_centerline_yaw(ctrl_x(max(i-2, int32(1))), ctrl_y(max(i-2, int32(1))), ...
                                          ctrl_x(i-1), ctrl_y(i-1), cand_x, cand_y);

        safe_a = ~is_collision_segment(ctrl_x(i-1), ctrl_y(i-1), yaw_prev, cand_x, cand_y, yaw_a, occ_map);
        safe_b = ~is_collision_segment(cand_x, cand_y, yaw_a, ctrl_x(i+1), ctrl_y(i+1), yaw_b, occ_map);
        if safe_a && safe_b
            ctrl_x(i) = cand_x;
            ctrl_y(i) = cand_y;
        end
    end
end

[bs_x, bs_y, bs_len] = sample_cubic_bspline(ctrl_x, ctrl_y, plen);
if bs_len >= int32(2) && is_path_collision_free(bs_x, bs_y, bs_len, occ_map)
    out_x(:) = 0.0;
    out_y(:) = 0.0;
    out_yaw(:) = 0.0;
    out_dir(:) = int8(-1);
    for i = int32(1):bs_len
        out_x(i) = bs_x(i);
        out_y(i) = bs_y(i);
        out_dir(i) = int8(-1);
    end
    out_len = bs_len;
else
    out_len = plen;
    for i = int32(1):plen
        out_x(i) = ctrl_x(i);
        out_y(i) = ctrl_y(i);
        out_dir(i) = int8(-1);
    end
end

for i = int32(1):out_len
    if i < out_len
        out_yaw(i) = wrap_pi(atan2(out_y(i+1) - out_y(i), out_x(i+1) - out_x(i)) + pi);
    elseif i > int32(1)
        out_yaw(i) = out_yaw(i-1);
    else
        out_yaw(i) = in_yaw(1);
    end
end
end

function [bs_x, bs_y, bs_len] = sample_cubic_bspline(ctrl_x, ctrl_y, plen)
%#codegen
MAX_PATH = int32(300);
bs_x = zeros(1, MAX_PATH);
bs_y = zeros(1, MAX_PATH);
bs_len = int32(0);

if plen < int32(4)
    return;
end

bs_len = int32(1);
bs_x(1) = ctrl_x(1);
bs_y(1) = ctrl_y(1);

for i = int32(1):plen-int32(1)
    p0 = max(i - int32(1), int32(1));
    p1 = i;
    p2 = i + int32(1);
    p3 = min(i + int32(2), plen);
    for sj = int32(1):int32(2)
        if bs_len >= MAX_PATH
            break;
        end
        u = double(sj) / 2.0;
        u2 = u * u;
        u3 = u2 * u;
        b0 = (-u3 + 3.0*u2 - 3.0*u + 1.0) / 6.0;
        b1 = ( 3.0*u3 - 6.0*u2 + 4.0) / 6.0;
        b2 = (-3.0*u3 + 3.0*u2 + 3.0*u + 1.0) / 6.0;
        b3 = u3 / 6.0;
        bs_len = bs_len + int32(1);
        bs_x(bs_len) = b0*ctrl_x(p0) + b1*ctrl_x(p1) + b2*ctrl_x(p2) + b3*ctrl_x(p3);
        bs_y(bs_len) = b0*ctrl_y(p0) + b1*ctrl_y(p1) + b2*ctrl_y(p2) + b3*ctrl_y(p3);
    end
end

bs_x(bs_len) = ctrl_x(plen);
bs_y(bs_len) = ctrl_y(plen);
end

function free = is_path_collision_free(px, py, plen, occ_map)
%#codegen
free = true;
for i = int32(1):plen-int32(1)
    yaw1 = reverse_centerline_yaw(px(max(i-1, int32(1))), py(max(i-1, int32(1))), ...
                                  px(i), py(i), px(i+1), py(i+1));
    yaw2 = reverse_centerline_yaw(px(i), py(i), px(i+1), py(i+1), ...
                                  px(min(i+2, plen)), py(min(i+2, plen)));
    if is_collision_segment(px(i), py(i), yaw1, px(i+1), py(i+1), yaw2, occ_map)
        free = false;
        return;
    end
end
end

function free = is_candidate_path_safe(px, py, pyaw, plen, occ_map)
%#codegen
free = true;
for i = int32(1):plen-int32(1)
    if is_collision_segment(px(i), py(i), pyaw(i), px(i+1), py(i+1), pyaw(i+1), occ_map)
        free = false;
        return;
    end
end
end

function [out_x, out_y, out_yaw, out_dir, out_len] = ...
    smooth_forward_bspline_constrained(in_x, in_y, in_yaw, in_dir, in_len, occ_map)
%#codegen
MAX_PATH = int32(300);
out_x = in_x;
out_y = in_y;
out_yaw = in_yaw;
out_dir = in_dir;
out_len = in_len;

plen = in_len;
if plen < int32(5)
    return;
end
if plen > MAX_PATH
    plen = MAX_PATH;
end

all_forward = true;
for i = int32(1):plen
    if in_dir(i) < int8(0)
        all_forward = false;
        break;
    end
end
if ~all_forward
    return;
end

ctrl_x = in_x;
ctrl_y = in_y;
alpha = 0.30;

for iter = int32(1):int32(5)
    for i = int32(2):plen-int32(1)
        cand_x = ctrl_x(i) + alpha * (0.5 * (ctrl_x(i-1) + ctrl_x(i+1)) - ctrl_x(i));
        cand_y = ctrl_y(i) + alpha * (0.5 * (ctrl_y(i-1) + ctrl_y(i+1)) - ctrl_y(i));

        yaw_a = forward_centerline_yaw(ctrl_x(i-1), ctrl_y(i-1), cand_x, cand_y, ctrl_x(i+1), ctrl_y(i+1));
        yaw_b = forward_centerline_yaw(cand_x, cand_y, ctrl_x(i+1), ctrl_y(i+1), ...
                                       ctrl_x(min(i+2, plen)), ctrl_y(min(i+2, plen)));
        yaw_prev = forward_centerline_yaw(ctrl_x(max(i-2, int32(1))), ctrl_y(max(i-2, int32(1))), ...
                                          ctrl_x(i-1), ctrl_y(i-1), cand_x, cand_y);

        safe_a = ~is_collision_segment(ctrl_x(i-1), ctrl_y(i-1), yaw_prev, cand_x, cand_y, yaw_a, occ_map);
        safe_b = ~is_collision_segment(cand_x, cand_y, yaw_a, ctrl_x(i+1), ctrl_y(i+1), yaw_b, occ_map);
        if safe_a && safe_b
            ctrl_x(i) = cand_x;
            ctrl_y(i) = cand_y;
        end
    end
end

[bs_x, bs_y, bs_len] = sample_cubic_bspline(ctrl_x, ctrl_y, plen);
if bs_len >= int32(2) && is_forward_path_collision_free(bs_x, bs_y, bs_len, occ_map)
    out_x(:) = 0.0;
    out_y(:) = 0.0;
    out_yaw(:) = 0.0;
    out_dir(:) = int8(1);
    for i = int32(1):bs_len
        out_x(i) = bs_x(i);
        out_y(i) = bs_y(i);
        out_dir(i) = int8(1);
    end
    out_len = bs_len;
else
    out_len = plen;
    for i = int32(1):plen
        out_x(i) = ctrl_x(i);
        out_y(i) = ctrl_y(i);
        out_dir(i) = int8(1);
    end
end

for i = int32(1):out_len
    if i < out_len
        out_yaw(i) = atan2(out_y(i+1) - out_y(i), out_x(i+1) - out_x(i));
    elseif i > int32(1)
        out_yaw(i) = out_yaw(i-1);
    else
        out_yaw(i) = in_yaw(1);
    end
end
end

function free = is_forward_path_collision_free(px, py, plen, occ_map)
%#codegen
free = true;
for i = int32(1):plen-int32(1)
    yaw1 = forward_centerline_yaw(px(max(i-1, int32(1))), py(max(i-1, int32(1))), ...
                                  px(i), py(i), px(i+1), py(i+1));
    yaw2 = forward_centerline_yaw(px(i), py(i), px(i+1), py(i+1), ...
                                  px(min(i+2, plen)), py(min(i+2, plen)));
    if is_collision_segment(px(i), py(i), yaw1, px(i+1), py(i+1), yaw2, occ_map)
        free = false;
        return;
    end
end
end

function yaw = forward_centerline_yaw(x_prev, y_prev, x_now, y_now, x_next, y_next)
%#codegen
dx = x_next - x_prev;
dy = y_next - y_prev;
if abs(dx) + abs(dy) < 1.0e-6
    dx = x_next - x_now;
    dy = y_next - y_now;
end
if abs(dx) + abs(dy) < 1.0e-6
    yaw = 0.0;
else
    yaw = atan2(dy, dx);
end
end

function yaw = reverse_centerline_yaw(x_prev, y_prev, x_now, y_now, x_next, y_next)
%#codegen
dx = x_next - x_prev;
dy = y_next - y_prev;
if abs(dx) + abs(dy) < 1.0e-6
    dx = x_next - x_now;
    dy = y_next - y_now;
end
if abs(dx) + abs(dy) < 1.0e-6
    yaw = 0.0;
else
    yaw = wrap_pi(atan2(dy, dx) + pi);
end
end

%% =====================================================================
%% Local helpers — indexed binary min-heap (open set)
%% =====================================================================

function [heap, heap_pos, heap_size] = heap_push(heap, heap_pos, heap_size, node, nf)
%#codegen
% Insert node (with priority nf(node)) into the heap, then sift it up.
heap_size = heap_size + int32(1);
heap(heap_size) = node;
heap_pos(node)  = heap_size;
[heap, heap_pos] = heap_sift_up(heap, heap_pos, heap_size, nf);
end

function [heap, heap_pos, heap_size] = heap_remove_top(heap, heap_pos, heap_size, nf)
%#codegen
% Remove the min element (heap(1)).  Caller reads heap(1) before calling.
heap_pos(heap(1)) = int32(0);            % old top leaves the heap
last = heap(heap_size);
heap_size = heap_size - int32(1);
if heap_size >= int32(1)
    heap(1) = last;
    heap_pos(last) = int32(1);
    pos = int32(1);
    while true
        l = int32(2) * pos;
        r = l + int32(1);
        smallest = pos;
        if l <= heap_size && heap_less(heap(l), heap(smallest), nf)
            smallest = l;
        end
        if r <= heap_size && heap_less(heap(r), heap(smallest), nf)
            smallest = r;
        end
        if smallest == pos
            break;
        end
        tmp = heap(pos); heap(pos) = heap(smallest); heap(smallest) = tmp;
        heap_pos(heap(pos))      = pos;
        heap_pos(heap(smallest)) = smallest;
        pos = smallest;
    end
end
end

function [heap, heap_pos] = heap_sift_up(heap, heap_pos, pos, nf)
%#codegen
% Move the node at slot pos toward the root until heap order is restored.
while pos > int32(1)
    parent = max(int32(1), bitshift(pos, -1));   % floor(pos/2); max() keeps it >=1 for codegen
    if heap_less(heap(pos), heap(parent), nf)
        tmp = heap(parent); heap(parent) = heap(pos); heap(pos) = tmp;
        heap_pos(heap(parent)) = parent;
        heap_pos(heap(pos))    = pos;
        pos = parent;
    else
        break;
    end
end
end

function [heap, heap_pos] = heap_resift(heap, heap_pos, heap_size, pos, nf)
%#codegen
% Restore heap order after the key at slot `pos` changed in EITHER direction.
% A dup relaxation lowers g but can RAISE f (the overwritten pose may have a
% worse heuristic), so the node may need to move down, not just up.  Sift up
% first; if it did not move, sift down.
if pos < int32(1)        % not in heap (defensive) -> nothing to do; bounds index for codegen
    return;
end
moved_up = false;
while pos > int32(1)
    parent = max(int32(1), bitshift(pos, -1));   % floor(pos/2); max() keeps it >=1 for codegen
    if heap_less(heap(pos), heap(parent), nf)
        tmp = heap(parent); heap(parent) = heap(pos); heap(pos) = tmp;
        heap_pos(heap(parent)) = parent;
        heap_pos(heap(pos))    = pos;
        pos = parent;
        moved_up = true;
    else
        break;
    end
end
if ~moved_up
    while true
        l = int32(2) * pos;
        r = l + int32(1);
        smallest = pos;
        if l <= heap_size && heap_less(heap(l), heap(smallest), nf)
            smallest = l;
        end
        if r <= heap_size && heap_less(heap(r), heap(smallest), nf)
            smallest = r;
        end
        if smallest == pos
            break;
        end
        tmp = heap(pos); heap(pos) = heap(smallest); heap(smallest) = tmp;
        heap_pos(heap(pos))      = pos;
        heap_pos(heap(smallest)) = smallest;
        pos = smallest;
    end
end
end

function tf = heap_less(a, b, nf)
%#codegen
% True if node a outranks node b: smaller nf, ties broken by smaller node
% index.  This matches the original strict-< linear scan, which kept the
% lowest-index node among equal-f candidates -> selection order is identical.
fa = nf(a); fb = nf(b);
if fa < fb
    tf = true;
elseif fa > fb
    tf = false;
else
    tf = (a < b);
end
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
%% Local helpers — PD longitudinal controller
%% =====================================================================

function [desired_ax, tire_angle, vc_selector, vc_gas, vc_brake] = ...
    control_with_shift_delay_local(ego_v, target_v, steer_cmd, dir_cmd)
%#codegen
% EV6 parking control uses direct VC pedals. AccelCtrl.DesiredAx is kept at 0.
% Gear changes wait at standstill before VC.SelectCtrl is switched.
persistent active_selector switch_count

ego_v = ego_v(1);
target_v = abs(target_v(1));
steer_cmd = steer_cmd(1);
dir_cmd = dir_cmd(1);

if isempty(active_selector)
    active_selector = 1.0;
    switch_count = int32(0);
end

desired_ax = 0.0;
tire_angle = steer_cmd;

speed_abs = abs(ego_v);

requested_selector = 1.0;
if dir_cmd < 0.0
    requested_selector = -1.0;
end

if requested_selector ~= active_selector
    tire_angle = 0.0;
    target_v = 0.0;

    if speed_abs < 0.12
        switch_count = switch_count + int32(1);
        if switch_count > int32(50)
            active_selector = requested_selector;
            switch_count = int32(0);
        end
    else
        switch_count = int32(0);
    end
else
    switch_count = int32(0);
end

vc_selector = active_selector;
vc_gas = 0.0;
vc_brake = 0.0;

if requested_selector ~= active_selector
    vc_gas = 0.0;
    vc_brake = 0.45;
    return;
end

if target_v < 0.05
    vc_gas = 0.0;
    vc_brake = 0.45;
    return;
end

if speed_abs > target_v + 0.45
    vc_gas = 0.0;
    vc_brake = 0.35;
elseif speed_abs < target_v - 0.30
    vc_brake = 0.0;
    if vc_selector < 0.0
        vc_gas = 0.18;
    else
        vc_gas = 0.24;
    end
else
    vc_brake = 0.0;
    if vc_selector < 0.0
        vc_gas = 0.06;
    else
        vc_gas = 0.09;
    end
end
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
    V_MAX_REV = 0.6;     % reverse top speed                                 [tunable]
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

%% =====================================================================
%% Local helpers — Reeds-Shepp analytic shot (CSC subset)
%% =====================================================================

function [px, py, pyaw, pdir, plen, ok] = rs_shot(sx, sy, syaw, gx, gy, gyaw, occ_map, R, ds)
%#codegen
MAX_PATH = int32(300);
px   = zeros(1, MAX_PATH);
py   = zeros(1, MAX_PATH);
pyaw = zeros(1, MAX_PATH);
pdir = zeros(1, MAX_PATH, 'int8');
plen = int32(0);
ok   = false;

patterns = [...
    +1, -1, +1;
    +1, -1, -1;
    -1, -1, +1;
    -1, -1, -1];

best_len = 1.0e18;
for p = 1:size(patterns, 1)
    t1 = patterns(p, 1);
    sd = patterns(p, 2);
    t2 = patterns(p, 3);
    [c_ok, c_px, c_py, c_pyaw, c_pdir, c_plen, c_len] = ...
        try_csc(sx, sy, syaw, gx, gy, gyaw, R, ds, t1, sd, t2, occ_map, MAX_PATH);
    if c_ok && c_len < best_len
        best_len = c_len;
        px   = c_px;
        py   = c_py;
        pyaw = c_pyaw;
        pdir = c_pdir;
        plen = c_plen;
        ok   = true;
    end
end
end

function [ok, px, py, pyaw, pdir, plen, total_len] = try_csc(sx, sy, syaw, ...
        gx, gy, gyaw, R, ds, t1, sd, t2, occ_map, MAX_PATH)
%#codegen
ok = false;
px = zeros(1, MAX_PATH); py = zeros(1, MAX_PATH);
pyaw = zeros(1, MAX_PATH); pdir = zeros(1, MAX_PATH, 'int8');
plen = int32(0);
total_len = 1.0e18;

c1x = sx - R * sin(syaw) * t1;
c1y = sy + R * cos(syaw) * t1;
c2x = gx - R * sin(gyaw) * t2;
c2y = gy + R * cos(gyaw) * t2;

dxc = c2x - c1x;
dyc = c2y - c1y;
d_cc = hypot(dxc, dyc);

if t1 == t2
    if d_cc < 1.0e-6
        return;
    end
    seg_len = d_cc;
    alpha = atan2(dyc, dxc);
    tp1x = c1x + R * sin(alpha) * t1;
    tp1y = c1y - R * cos(alpha) * t1;
    tp2x = tp1x + seg_len * cos(alpha);
    tp2y = tp1y + seg_len * sin(alpha);
    tangent_yaw = alpha;
else
    if d_cc < 2.0 * R
        return;
    end
    seg_len = sqrt(d_cc * d_cc - 4.0 * R * R);
    alpha = atan2(dyc, dxc);
    beta = atan2(2.0 * R, seg_len) * t1;
    tan_dir = alpha - beta;
    tp1x = c1x + R * sin(tan_dir) * t1;
    tp1y = c1y - R * cos(tan_dir) * t1;
    tp2x = tp1x + seg_len * cos(tan_dir);
    tp2y = tp1y + seg_len * sin(tan_dir);
    tangent_yaw = tan_dir;
end

arc1_yaw_start = wrap_pi(syaw);
arc1_yaw_end   = wrap_pi(tangent_yaw);
arc1_len_signed = arc_length_along_circle(arc1_yaw_start, arc1_yaw_end, t1) * R;

arc2_yaw_start = wrap_pi(tangent_yaw);
arc2_yaw_end   = wrap_pi(gyaw);
arc2_len_signed = arc_length_along_circle(arc2_yaw_start, arc2_yaw_end, t2) * R;

total_len = abs(arc1_len_signed) + seg_len + abs(arc2_len_signed);

n_arc1 = max(int32(1), int32(ceil(abs(arc1_len_signed) / ds)));
n_seg  = max(int32(1), int32(ceil(seg_len / ds)));
n_arc2 = max(int32(1), int32(ceil(abs(arc2_len_signed) / ds)));
total_n = n_arc1 + n_seg + n_arc2 + int32(1);
if total_n > MAX_PATH
    return;
end

idx = int32(1);
px(idx) = sx; py(idx) = sy; pyaw(idx) = syaw; pdir(idx) = int8(sd);

for k = int32(1):n_arc1
    s = double(k) / double(n_arc1);
    yawk = arc1_yaw_start + s * (arc1_len_signed / R);
    xk = c1x + R * sin(yawk) * t1;
    yk = c1y - R * cos(yawk) * t1;
    idx = idx + int32(1);
    px(idx) = xk; py(idx) = yk; pyaw(idx) = wrap_pi(yawk); pdir(idx) = int8(sd);
end

for k = int32(1):n_seg
    s = double(k) / double(n_seg);
    xk = tp1x + s * (tp2x - tp1x);
    yk = tp1y + s * (tp2y - tp1y);
    idx = idx + int32(1);
    px(idx) = xk; py(idx) = yk; pyaw(idx) = tangent_yaw; pdir(idx) = int8(sd);
end

for k = int32(1):n_arc2
    s = double(k) / double(n_arc2);
    yawk = arc2_yaw_start + s * (arc2_len_signed / R);
    xk = c2x + R * sin(yawk) * t2;
    yk = c2y - R * cos(yawk) * t2;
    idx = idx + int32(1);
    px(idx) = xk; py(idx) = yk; pyaw(idx) = wrap_pi(yawk); pdir(idx) = int8(sd);
end

plen = idx;

for i = int32(1):plen-int32(1)
    if is_collision_segment(px(i), py(i), pyaw(i), px(i+1), py(i+1), pyaw(i+1), occ_map)
        plen = int32(0);
        return;
    end
end

ok = true;
end

function len = arc_length_along_circle(yaw_start, yaw_end, turn_sign)
%#codegen
d = wrap_pi(yaw_end - yaw_start);
if turn_sign > 0
    if d < 0
        d = d + 2.0 * pi;
    end
else
    if d > 0
        d = d - 2.0 * pi;
    end
end
len = d;
end

%% =====================================================================
%% Local helpers — Heuristic + clearance grids
%% =====================================================================

function h_grid = compute_grid_heuristic(occ_map, gx, gy)
%#codegen
c = map_const_local();
N = double(c.N);
res = c.RES;
N_int = int32(N);

INF = single(1.0e9);
h_grid = INF * ones(N, N, 'single');

gc = floor((gx - c.X_MIN) / res) + 1;
gr = floor((c.Y_MAX - gy) / res) + 1;
if gc < 1 || gc > N || gr < 1 || gr > N
    return;
end
if occ_map(int32(gr), int32(gc)) > 0
    return;
end

MAX_Q = int32(8 * N * N);
qr = zeros(MAX_Q, 1, 'int32');
qc = zeros(MAX_Q, 1, 'int32');
qhead = int32(1);
qtail = int32(2);

h_grid(int32(gr), int32(gc)) = single(0);
qr(1) = int32(gr);
qc(1) = int32(gc);

DR = int32([-1 -1 -1  0  0  1  1  1]);
DC = int32([-1  0  1 -1  1 -1  0  1]);
SQ2 = single(sqrt(2.0));
COSTS = single([SQ2 1 SQ2 1 1 SQ2 1 SQ2]) * single(res);

while qhead < qtail
    r = qr(qhead);
    cc = qc(qhead);
    qhead = qhead + int32(1);

    h_here = h_grid(r, cc);

    for k = int32(1):int32(8)
        nr = r + DR(k);
        nc = cc + DC(k);
        if nr < 1 || nr > N_int || nc < 1 || nc > N_int
            continue;
        end
        if occ_map(nr, nc) > 0
            continue;
        end
        new_h = h_here + COSTS(k);
        if new_h < h_grid(nr, nc)
            h_grid(nr, nc) = new_h;
            if qtail <= MAX_Q
                qr(qtail) = nr;
                qc(qtail) = nc;
                qtail = qtail + int32(1);
            end
        end
    end
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
        if occ_map(nr, nc) > 0
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
%% Local helpers — Hybrid A* primitives, grid lookup, math
%% =====================================================================

function inflated = inflate_map_for_planning(occ_map, radius)
%#codegen
c = map_const_local();
inflated = uint8(occ_map);
if radius <= 1.0e-6
    return;
end

rad_cells = int32(ceil(radius / c.RES));
r2 = radius * radius;

for row = int32(1):c.N
    for col = int32(1):c.N
        if occ_map(row, col) > 0
            r0 = max(int32(1), row - rad_cells);
            r1 = min(c.N, row + rad_cells);
            c0 = max(int32(1), col - rad_cells);
            c1 = min(c.N, col + rad_cells);
            for rr = r0:r1
                dy = double(rr - row) * c.RES;
                for cc = c0:c1
                    dx = double(cc - col) * c.RES;
                    if dx * dx + dy * dy <= r2
                        inflated(rr, cc) = uint8(1);
                    end
                end
            end
        end
    end
end
end

function inside = pose_in_goal_box(ex, ey, eyaw, bx, by, byaw, ...
                                    box_l, box_w, box_tol, ego_l, ego_w)
%#codegen
c_e = cos(eyaw); s_e = sin(eyaw);
c_b = cos(byaw); s_b = sin(byaw);

half_w = ego_w * 0.5;
box_half_w = box_w * 0.5;

cx_loc = [0.0,   0.0,   ego_l, ego_l];
cy_loc = [+half_w, -half_w, +half_w, -half_w];

inside = true;
for k = 1:4
    gx_c = ex + c_e * cx_loc(k) - s_e * cy_loc(k);
    gy_c = ey + s_e * cx_loc(k) + c_e * cy_loc(k);
    dx = gx_c - bx;
    dy = gy_c - by;
    lx =  c_b * dx + s_b * dy;
    ly = -s_b * dx + c_b * dy;
    if lx < -box_tol || lx > box_l + box_tol || ...
       ly < -(box_half_w + box_tol) || ly > (box_half_w + box_tol)
        inside = false;
        return;
    end
end
end

function h = lookup_h(x, y, h_grid)
%#codegen
N_grid = 200;
RES = 0.5;
X_MIN = 0.0;
X_MAX = 100.0;
Y_MIN = -100.0;
Y_MAX = 0.0;
if x < X_MIN || x > X_MAX || y < Y_MIN || y > Y_MAX
    h = double(1.0e9);
    return;
end
col = floor((x - X_MIN) / RES) + 1;
row = floor((Y_MAX - y) / RES) + 1;
if row < 1 || row > N_grid || col < 1 || col > N_grid
    h = double(1.0e9);
    return;
end
h = double(h_grid(int32(row), int32(col)));
end

function [nx_o, ny_o, nyaw_o] = bicycle_step(x, y, yaw, steer, ds, L)
%#codegen
% State x/y is the rear-bumper center. The single-track model propagates the
% rear-axle center, so convert bumper -> rear axle -> bumper each primitive.
c = map_const_local();
a = c.REAR_AXLE_FROM_REAR_BUMPER;

rx = x + a * cos(yaw);
ry = y + a * sin(yaw);

if abs(steer) < 1.0e-6
    rx_n = rx + ds * cos(yaw);
    ry_n = ry + ds * sin(yaw);
    nyaw_o = yaw;
else
    beta = ds * tan(steer) / L;
    R    = L / tan(steer);
    rx_n = rx + R * (sin(yaw + beta) - sin(yaw));
    ry_n = ry - R * (cos(yaw + beta) - cos(yaw));
    nyaw_o = yaw + beta;
end
nyaw_o = wrap_pi(nyaw_o);

nx_o = rx_n - a * cos(nyaw_o);
ny_o = ry_n - a * sin(nyaw_o);
end

function col = is_collision_segment(x1, y1, yaw1, x2, y2, yaw2, occ_map)
%#codegen
col = false;
for k = int32(0):int32(12)
    t = double(k) / 12.0;
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
c = map_const_local();
c_yaw = cos(yaw);
s_yaw = sin(yaw);
occ = false;

sample_len = c.EGO_L + c.EGO_FRONT_SAFETY_MARGIN + c.EGO_REAR_SAFETY_MARGIN;
sample_width = c.EGO_W + 2.0 * c.EGO_WIDTH_SAFETY_MARGIN;
num_s = int32(ceil(sample_len / c.FOOTPRINT_LENGTH_SAMPLE_DS)) + int32(1);
num_l = int32(ceil(sample_width / c.FOOTPRINT_WIDTH_SAMPLE_DS)) + int32(1);
if num_s > int32(16); num_s = int32(16); end
if num_l > int32(8);  num_l = int32(8);  end

for si = int32(1):num_s
    if num_s <= int32(1)
        s = 0.0;
    else
        s = -c.EGO_REAR_SAFETY_MARGIN + sample_len * double(si - int32(1)) / double(num_s - int32(1));
    end
    for li = int32(1):num_l
        if num_l <= int32(1)
            lateral = 0.0;
        else
            lateral = -0.5 * sample_width + sample_width * double(li - int32(1)) / double(num_l - int32(1));
        end
        px = x + s * c_yaw - lateral * s_yaw;
        py = y + s * s_yaw + lateral * c_yaw;
        if is_occupied_point(px, py, occ_map)
            occ = true;
            return;
        end
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

function a = wrap_pi(a)
%#codegen
while a > pi;  a = a - 2.0 * pi; end
while a < -pi; a = a + 2.0 * pi; end
end

function a = wrap_2pi(a)
%#codegen
while a < 0.0;       a = a + 2.0 * pi; end
while a >= 2.0 * pi; a = a - 2.0 * pi; end
end

function d = angle_diff(a, b)
%#codegen
d = wrap_pi(a - b);
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

function c = map_const_local()
%#codegen
c.N        = int32(200);
c.RES      = 0.5;
c.X_MIN    = 0.0;
c.X_MAX    = 100.0;
c.Y_MIN    = -100.0;
c.Y_MAX    = 0.0;
c.TRUCK_W  = 2.48;
c.TRUCK_L  = 11.5;
c.EGO_W    = 1.9;
c.EGO_L    = 4.7;
c.WHEELBASE = 2.8;
c.REAR_AXLE_FROM_REAR_BUMPER = 0.95;
c.SAFETY_MARGIN = 0.8;
c.PLANNER_EXTRA_MARGIN = 0.20;
c.PATH_WIDTH_MARGIN = 0.35;
c.EGO_WIDTH_SAFETY_MARGIN = 0.25;
c.EGO_FRONT_SAFETY_MARGIN = 0.45;
c.EGO_REAR_SAFETY_MARGIN = 0.35;
c.CENTERLINE_SAMPLE_DS = 0.45;
c.FOOTPRINT_LENGTH_SAMPLE_DS = 0.55;
c.FOOTPRINT_WIDTH_SAMPLE_DS = 0.45;
c.CLEAR_MAX = 3.0;
c.W_CLEAR   = 2.0;
c.PARK_BOX_L = 6.0;
c.PARK_BOX_W = 3.0;
c.PARK_TOL   = 0.05;
end
