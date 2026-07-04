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
cfg.holdout     = 21;                     % pp_obstacles row kept FREE as the parking
                                          %   target (held-out car). 21 = T20 (corner, far bottom-right; neighbor T19). 0 = none.

% --- occupancy grid --------------------------------------------------------
cfg.n           = 200;                    % cells/side (occ_map [200 200]) -- keep 200: EML occ port is fixed
cfg.margin      = 0.30;                   % obstacle footprint safety inflation [m] (buffer for tracking error)
cfg.bounds      = [-5 48 -45.5 6];        % world planning rect [xmin xmax ymin ymax] = lot + NORTH entrance approach
                                          %   (res 53/200=0.265 x 51.5/200=0.258 m)
cfg.lot         = [5.61 47.40 -44.88 -4.09]; % drivable parking-lot rectangle [xmin xmax ymin ymax] -- from USER FIELD-MEASURED corners (5.61,-44.88)/(47.40,-44.84)/(47.38,-4.09)/(5.69,-4.09); excludes entrance road (pp_entrance_path carves that). Carved FREE (inset edge_margin) by pp_generate_map.
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
cfg.v_fwd       = 2.0;                    % forward cruise speed [m/s] (raised 1.0->2.0 to cut the long fwd approach time; forward tracks setpoint well in logs. v_emax raised in step.)
cfg.v_rev       = 0.4;                    % reverse cruise speed [m/s] (EXP 0.2->0.4: baseline log = plant reaches 0.52 in rev yet 40% idle at <=0.2 setpoint; raise to cut stall/idle & time. Validate in CarMaker.)
cfg.v_creep     = 0.10;                   % min creep speed while cruising [m/s]
cfg.v_stop      = 0.10;                   % treated as stopped [m/s]
cfg.kp_v        = 0.6;                    % longitudinal P gain (outer speed loop)
cfg.ax_max      = 0.6;                    % accel/brake command limit [m/s^2]
cfg.ax_min      = -1.5;                   % emergency decel/brake limit [m/s^2]
cfg.v_emax      = 2.8;                    % over-speed guard -> emergency brake [m/s] (raised 1.2->2.8 to clear v_fwd=2.0 + overshoot; still catches genuine runaway; forward-only guard so reverse unaffected)
cfg.v_obrake    = 0.40;                   % SAFETY-NET margin over target -> hard brake [m/s] (PI handles normal regulation)
cfg.ax_brake    = 1.5;                    % safety-net brake demand magnitude [m/s^2] (gear-dir: -ve = brake)
cfg.stuck_v     = 0.12;                   % gear-dir speed below which a moving-commanded reverse is "stalled" [m/s]
cfg.unstick_ax  = 0.45;                   % gas-dir desired_ax floor to break static friction when stalled in R [m/s^2]
                                          %   (EXP: instrumented log showed desired_ax<=0.24 -> VC_Gas<=0.23 -> stuck up to ~15 s)
cfg.d_slow      = 5.0;                    % slow-down distance before cusp/goal [m] (weak brake -> decelerate early)
cfg.cusp_reach  = 0.30;                   % distance to register a cusp reached [m]
cfg.goal_reach  = 0.08;                   % path-end position tol [m]
cfg.goal_near   = 2.0;                     % overshoot (sprog<=0) only ends tracking within this dist of goal [m]
                                          %   (prevents premature stop on long cusp-free/all-forward paths, e.g. T14)
cfg.max_rounds  = 4;                      % max RS corrective rounds in ALIGN (few shifts)
cfg.ok_corner   = 0.15;                   % CORRECT good-enough stop (checked EVERY step): worst ego-vs-goal corner < this [m] (success<0.2, margin for coast)
cfg.div_margin  = 0.05;                   % CORRECT anti-divergence: stop if a round's corner error exceeds best-seen by this [m]

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
