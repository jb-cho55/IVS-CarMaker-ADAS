% ===== VERIFY RUNTIME MAP from EDITED charts (lot bottom -38) =====
map0 = pp_generate_map([],[],[]);
occ  = pp_add_obstacle(map0,[],[]);
n=200; b=[-5 48 -45.5 6]; resx=(b(2)-b(1))/n; resy=(b(4)-b(3))/n;
g2i=@(x,y) deal(min(max(floor((x-b(1))/resx)+1,1),n), min(max(floor((y-b(3))/resy)+1,1),n));
[cx,cy]=g2i(3.417,-40.238); crash=occ(cy,cx);
[gx2,gy2]=g2i(9,-6);       goalc=occ(gy2,gx2);
fprintf('free fraction=%.1f%%\n',100*mean(occ(:)==0));
fprintf('crash pt (3.42,-40.24): occ=%d  (expect 1=occupied/wall)\n',crash);
fprintf('goal  pt (9,-6)       : occ=%d  (expect 0=free)\n',goalc);
% goal-slot ego footprint free-cell count
H=pi/2; aL=4.68; bw=1.88/2; co=cos(-H);si=sin(-H);
xs=9+co*[aL;aL;0;0]-si*[bw;-bw;-bw;bw]; ys=-6+si*[aL;aL;0;0]+co*[bw;-bw;-bw;bw];
[XX,YY]=meshgrid(b(1)+((1:n)-0.5)*resx, b(3)+((1:n)-0.5)*resy);
inpoly=inpolygon(XX,YY,xs,ys); occc=sum(occ(inpoly)>0);
fprintf('goal ego footprint occupied cells=%d of %d (expect 0)\n',occc,sum(inpoly(:)));
% min free-y near corridor exit (x in [-1,3])
col=XX>=-1 & XX<=3 & occ==0; ycol=YY(col); fprintf('min free Y in corridor-exit band=%.2f (wall below should be occupied)\n', min(ycol));
assignin('base','occ_runtime',occ); disp('MAP_VERIFY_DONE');

function mapMatrix = generate_map_(map_boundary, traffic_info, traffic_size)
%GENERATE_MAP_  Self-contained version of the "MATLAB Function1" block.
%   Source model : Models/Day4_5/Day4_5_Scenario_1.slx
%   Block path   : CarMaker/Subsystem/Day4 & 5/Scenario 1/MATLAB Function1
%
%   The block body only calls the pp_* helper; this file additionally
%   bundles every pp_*.m it transitively depends on as local functions,
%   so it runs standalone without dev/day4_5/pp_*.m on the MATLAB path.
%
%   Bundled dependencies (5): pp_generate_map, pp_cfg, pp_entrance_path, pp_grid2world, pp_world2grid
mapMatrix = pp_generate_map(map_boundary, traffic_info, traffic_size);
end

% ========================================================================
%  Local dependency functions (bundled verbatim from dev/day4_5/pp_*.m).
%  Do not edit here; edit the pp_*.m sources and re-bundle.
% ========================================================================

function mapMatrix = pp_generate_map(map_boundary, traffic_info, traffic_size) %#ok<INUSD>
%#codegen
% PP_GENERATE_MAP  Base occupancy grid for the parking area (M2 logic).
%   Output mapMatrix is n-by-n, indexed (iy, ix): 0 = free, 1 = occupied.
%
%   Drivable space = the parking-lot rectangle (cfg.lot) UNION the entrance-road
%   corridor (route5 centerline +- halfw, from pp_entrance_path), each inset by
%   cfg.edge_margin. Everything else inside cfg.bounds is OCCUPIED, so the planner
%   cannot cut across non-road area. Static parked cars are stamped afterwards by
%   pp_add_obstacle.
%
%   PERFORMANCE: the map is STATIC, so it is computed ONCE and cached in a
%   persistent (this block runs every sim step; the corridor distance-carve must
%   not run per-step). Codegen rule: the persistent is assigned inside the
%   `if isempty(...)` guard BEFORE it is read. Cache clears at sim start and by
%   `clear functions` after editing cfg / pp_entrance_path.
%
%   map_boundary / traffic_info / traffic_size are accepted only for MATLAB
%   Function block signature compatibility (intentionally unused here).
persistent BASE
if isempty(BASE)
    cfg = pp_cfg();
    n = cfg.n; b = cfg.bounds; em = cfg.edge_margin;
    M = ones(n, n);                                 % default OCCUPIED; carve drivable area free

    % --- carve the parking-lot rectangle FREE (inset by the edge safety margin) -
    lot = cfg.lot;
    [lx0, ly0] = pp_world2grid(lot(1)+em, lot(3)+em, b, n);
    [lx1, ly1] = pp_world2grid(lot(2)-em, lot(4)-em, b, n);
    for ix = min(lx0,lx1):max(lx0,lx1)
        for iy = min(ly0,ly1):max(ly0,ly1)
            M(iy, ix) = 0;
        end
    end

    % --- carve the entrance-road corridor FREE (route5 centerline +- halfw) ----
    [WP, nW, halfw] = pp_entrance_path();
    halfw = max(halfw - em, 0.10);                  % inset corridor walls by the safety margin
    if nW >= 2
        xmnw = min(WP(:,1))-halfw; xmxw = max(WP(:,1))+halfw;
        ymnw = min(WP(:,2))-halfw; ymxw = max(WP(:,2))+halfw;
        [cx0, cy0] = pp_world2grid(xmnw, ymnw, b, n);
        [cx1, cy1] = pp_world2grid(xmxw, ymxw, b, n);
        for ix = min(cx0,cx1):max(cx0,cx1)
            for iy = min(cy0,cy1):max(cy0,cy1)
                [wx, wy] = pp_grid2world(ix, iy, b, n);
                dmin = inf;
                for s = 1:nW-1
                    d = seg_dist_(wx, wy, WP(s,1), WP(s,2), WP(s+1,1), WP(s+1,2));
                    if d < dmin, dmin = d; end
                end
                if dmin <= halfw, M(iy, ix) = 0; end
            end
        end
    end

    % --- occupied border ring (keeps the search bounded) ----------------------
    w = cfg.wall_cells;
    if w > 0
        M(1:w, :)     = 1;
        M(n-w+1:n, :) = 1;
        M(:, 1:w)     = 1;
        M(:, n-w+1:n) = 1;
    end

    BASE = M;
end
mapMatrix = BASE;
end

% ===== distance from point (px,py) to segment (ax,ay)-(bx,by) ==============
function d = seg_dist_(px, py, ax, ay, bx, by)
%#codegen
vx = bx-ax; vy = by-ay;
wx = px-ax; wy = py-ay;
vv = vx*vx + vy*vy;
if vv < 1e-9
    d = hypot(px-ax, py-ay); return;
end
t = (wx*vx + wy*vy) / vv;
if t < 0, t = 0; elseif t > 1, t = 1; end
qx = ax + t*vx; qy = ay + t*vy;
d = hypot(px-qx, py-qy);
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
cfg.lot         = [2 48 -38 -3];          % drivable parking-lot rectangle (bottom raised -45.5->-38: real road ends ~ -37..-40, wall-crash fix)
cfg.edge_margin = 0.20;                    % safety margin inset from the drivable-region boundary (lot edges + corridor walls) [m]
cfg.wall_cells  = 1;                      % occupied border-ring thickness [cells]

% --- Hybrid A* planner -----------------------------------------------------
cfg.plan_res    = 0.5;                    % closed-set position resolution [m]
cfg.nxc         = 106;                    % closed-set X bins  ~ (bounds_x)/plan_res = 53/0.5
cfg.nyc         = 103;                    % closed-set Y bins  ~ (bounds_y)/plan_res = 51.5/0.5
cfg.nth         = 72;                     % heading bins (5 deg)
cfg.ds          = 0.6;                    % motion step (arc length) [m]
cfg.n_steer     = 5;                      % steering samples (odd, symmetric)
cfg.max_nodes   = 60000;                  % expansion / node cap
cfg.max_path    = 2000;                   % max stored path points
cfg.pos_tol     = 0.7;                    % planner goal position tol [m] (M9 refines)
cfg.yaw_tol     = 12*pi/180;              % planner goal heading tol [rad]
cfg.w_rev       = 2.0;                    % reverse cost multiplier
cfg.w_switch    = 2.0;                    % direction-change penalty [~m]
cfg.w_steer     = 0.3;                    % steering magnitude penalty
cfg.h_weight    = 1.2;                    % heuristic weight (>=1 speeds search)
cfg.rs_shot_dist = 15.0;                  % try RS one-shot to goal within this dist [m]
cfg.rs_heur_dist = 25.0;                  % use RS (vs Euclidean) heuristic within [m]

% --- M7 path tracking + longitudinal control -------------------------------
cfg.Ld          = 1.5;                    % (legacy) nominal lookahead [m]
cfg.Ld_min      = 0.5;                    % min lookahead (tight low-speed tracking) [m]
cfg.Ld_max      = 1.8;                    % max lookahead [m]
cfg.kld         = 1.2;                    % lookahead gain vs |speed| (Ld = Ld_min + kld*|v|)
cfg.v_fwd       = 1.0;                    % forward cruise speed [m/s] (faster approach)
cfg.v_rev       = 0.2;                    % reverse cruise speed [m/s] (known-good; speed-up revisited separately)
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
cfg.max_rounds  = 4;                      % max RS corrective rounds in ALIGN (few shifts)

% --- M9 precise pose alignment (Astolfi pose regulation) -------------------
cfg.align_dist  = 3.0;                    % switch to alignment within this dist [m]
cfg.align_pos   = 0.08;                   % alignment done position tol [m]
cfg.align_yaw   = 2*pi/180;               % alignment done heading tol [rad]
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
end

function [WP, nW, halfw] = pp_entrance_path()
%#codegen
% PP_ENTRANCE_PATH  Baked centerline of the lot WEST/NORTH entrance road.
%   WP is nW-by-2 [x y] in the Car.Fr1 world frame (rear-bumper convention),
%   halfw = drivable corridor half-width [m]. pp_generate_map carves cells
%   within halfw of this polyline FREE (the rest of the approach stays occupied),
%   replacing the old "whole west strip is free" rectangle.
%   Source: src_cm4sl/map_data/final/ -- route5 centerline (final_map_waypoint_
%   raw_data.mat), width from left/right lane edges (final_map_raw_data.mat,
%   full road ~5.3 m). Northern approach truncated at y<=3.5 m to keep n=200
%   grid resolution; extend WP + cfg.bounds(4) north if the real start is higher.
WP = [ ...
   -0.851     3.445; ...
   -0.804     2.068; ...
   -0.830     0.629; ...
   -0.906    -0.866; ...
   -1.008    -2.409; ...
   -1.113    -3.990; ...
   -1.205    -5.600; ...
   -1.277    -7.231; ...
   -1.328    -8.875; ...
   -1.362   -10.527; ...
   -1.384   -12.178; ...
   -1.393   -13.819; ...
   -1.392   -15.446; ...
   -1.387   -17.060; ...
   -1.383   -18.655; ...
   -1.382   -20.198; ...
   -1.384   -21.659; ...
   -1.387   -23.036; ...
   -1.387   -24.337; ...
   -1.379   -25.578; ...
   -1.356   -26.774; ...
   -1.310   -27.938; ...
   -1.229   -29.080; ...
   -1.100   -30.200; ...
   -0.907   -31.294; ...
   -0.636   -32.353; ...
   -0.275   -33.376; ...
    0.187   -34.363; ...
    0.760   -35.309; ...
    1.441   -36.210; ...
    1.819   -36.641; ...
];
nW = size(WP,1);
halfw = 2.5;   % ~ measured half road width (full ~5.3 m); fits the 1.88 m car
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


function y = pp_add_obstacle(map, traffic_info, traffic_size) %#ok<INUSD>
%#codegen
% PP_ADD_OBSTACLE  Mark static parking obstacles onto the base map (M3 logic).
%   y = map with each obstacle footprint rasterized as occupied (1).
%
%   Obstacles are static & known, so they come from pp_obstacles() (rear-bumper
%   reference), inflated by cfg.margin on every side. The model's built-in
%   traffic reader (T01..T07) does NOT match the final lot's 21 cars, so
%   traffic_info / traffic_size are intentionally unused (kept for block
%   signature compatibility). Wire live traffic here only if the reader is
%   extended to all lot obstacles.
%
%   PERFORMANCE: the result is STATIC (cars never move; base map is the cached
%   constant from pp_generate_map), so it is computed ONCE and cached. Codegen
%   rule: the persistent is assigned inside the `if isempty(...)` guard BEFORE it
%   is read. Cache clears at sim start and by `clear functions`.
persistent FULL
if isempty(FULL)
    cfg = pp_cfg();
    n = cfg.n;
    b = cfg.bounds;
    yy = map;
    OBST = pp_obstacles();
    ahead  = cfg.obs_L + cfg.margin;
    behind = cfg.margin;
    halfw  = cfg.obs_W/2 + cfg.margin;
    for k = 1:size(OBST, 1)
        if k == cfg.holdout, continue; end          % keep the target slot free
        c = pp_rect_corners(OBST(k,1), OBST(k,2), OBST(k,3), ahead, behind, halfw);
        yy = pp_fill_rect(yy, c, b, n, 1);
    end
    FULL = yy;
end
y = FULL;
end


function map = pp_fill_rect(map, c, b, n, val)
%#codegen
% PP_FILL_RECT  Rasterize a filled (possibly rotated) rectangle onto the grid.
%   c   : 4-by-2 world corners. b,n : grid definition. val : value to write.
%   Cells whose center falls inside the quad are set to val. map is n-by-n,
%   indexed map(iy, ix) (row = Y index, col = X index).
ixv = zeros(1, 4); iyv = zeros(1, 4);
for k = 1:4
    [ixv(k), iyv(k)] = pp_world2grid(c(k,1), c(k,2), b, n);
end
imin = max(min(ixv), 1); imax = min(max(ixv), n);
jmin = max(min(iyv), 1); jmax = min(max(iyv), n);
for ix = imin:imax
    for iy = jmin:jmax
        [cx, cy] = pp_grid2world(ix, iy, b, n);
        if pp_pt_in_quad(cx, cy, c)
            map(iy, ix) = val;
        end
    end
end
end


function OBST = pp_obstacles()
%#codegen
% PP_OBSTACLES  Static parking-lot obstacle table for the mission.
%   Returns K-by-3 array, each row [x y yaw] = obstacle REAR-BUMPER center [m]
%   and heading [rad] (same convention as ego Car.Fr1).
%
% Values below are the IVS_Final_Project CarMaker scenario lot (T00..T15,
%   IPG_CompanyCar_2018_Blue), taken from Data/TestRun/IVS_Final_Project StartPos.
%   Footprint (length cfg.obs_L forward, width cfg.obs_W) is applied in
%   pp_add_obstacle; do NOT pre-offset here.
H = pi/2;
OBST = [ ...
    7.3  -28.7  -H;   % T00
   12.8   -6.8  -H;   % T01
   21.3   -6.6  -H;   % T02
   30.0   -6.5  -H;   % T03
   41.7   -6.3  -H;   % T04
   31.0  -33.5   H;   % T05
    7.0  -21.8   H;   % T06
   18.6  -21.8   H;   % T07
   24.3  -21.9   H;   % T08
   14.0  -21.9   H;   % T09
   38.8  -21.8   H;   % T10
   41.8  -21.8   H;   % T11
   11.0  -28.8  -H;   % T12
   24.3  -28.9  -H;   % T13
   41.6  -28.9  -H;   % T14
   15.0  -33.5   H];  % T15
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
