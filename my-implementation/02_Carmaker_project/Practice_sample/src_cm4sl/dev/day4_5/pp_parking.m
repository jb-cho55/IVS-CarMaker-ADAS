function [desired_ax, steer_fl, steer_fr, selector_ctrl] = pp_parking(ego_x, ego_y, ego_yaw, ego_v, start_point, finish_point, goal_yaw, occ_map) %#ok<INUSD>
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

persistent state PX PY PDIR NP seg gear replan nround bestce
if isempty(state)
    state = uint8(0);
    PX = zeros(MAXP,1); PY = zeros(MAXP,1); PDIR = zeros(MAXP,1);
    NP = 0; seg = 1; gear = cfg.GEAR_P; replan = false; nround = 0; bestce = inf;
end

[ex, ey] = pp_axle_from_fr1(ego_x, ego_y, ego_yaw, d_r); eth = ego_yaw;
gth = goal_yaw;
[gx, gy] = pp_axle_from_fr1(finish_point(1), finish_point(2), gth, d_r);

delta = 0.0; v_des = 0.0; hold = false;

switch state
    case uint8(0)                                       % INIT
        state = uint8(1); hold = true;

    case uint8(1)                                       % PLAN (one heavy step)
        [px, py, ~, pdir, np, ok] = pp_hybrid_astar(ex, ey, eth, gx, gy, gth, occ_map);
        if ok
            PX = px; PY = py; PDIR = pdir; NP = np; seg = 1; state = uint8(2);
        else
            state = uint8(4);
        end
        hold = true;

    case uint8(2)                                       % TRACK main path
        [delta, v_des, seg, done] = pp_track(ex, ey, eth, ego_v, PX, PY, PDIR, NP, seg, cfg);
        if done, state = uint8(3); replan = true; nround = 0; bestce = inf; end

    case uint8(3)                                       % CORRECT (committed RS rounds)
        if replan
            [Tr, sLr, ~, okr] = pp_reedsshepp(ex, ey, eth, gx, gy, gth, cfg.Rmin);
            if okr
                [rx, ry, ~, rdir, rns] = pp_rs_sample(ex, ey, eth, Tr, sLr, cfg.Rmin, cfg.ds, MAXP);
                if rns >= 2
                    PX = rx; PY = ry; PDIR = rdir; NP = rns; seg = 1;
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
            ce = corner_err_(ex, ey, eth, gx, gy, gth, cfg.foot_ahead, cfg.foot_behind, cfg.veh_halfW);
            if ce < cfg.ok_corner                                   % (2) good-enough -- checked EVERY step so we stop AS the
                state = uint8(4);                                   %     pose sweeps through the goal (a committed-RS round
            elseif done                                            %     tracks past the optimum, so a round-end check misses it)
                diverged = ce > bestce + cfg.div_margin;            % (3) this round left the pose worse than best-seen
                if ce < bestce, bestce = ce; end
                tight = hypot(gx-ex,gy-ey) < cfg.align_pos && abs(pp_angdiff(eth,gth)) < cfg.align_yaw;
                if tight || diverged || nround >= cfg.max_rounds
                    state = uint8(4);                               % tight / (3) diverged / round cap
                else
                    replan = true;                                  % next committed round
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
    v_drive = gdir*ego_v;                                  % speed in the gear (driving) direction
    desired_ax = cfg.kp_v*(abs(v_des) - v_drive);         % + = gas (gear dir), - = brake
    % breakaway FF (TRACK reverse only): a gentle reverse command can't overcome static friction at a
    % standstill (instrumented log: desired_ax<=0.24 -> VC_Gas<=0.23 -> stuck up to ~15 s). When the
    % tracker wants motion but we are stalled, floor the gas-dir command to break free; it disengages
    % once moving (v_drive>=stuck_v). GATED to TRACK (state 2): in CORRECT (state 3) the short
    % committed-RS corrections must stay gentle, else the FF overshoots them and the pose diverges.
    if state == uint8(2) && gear == cfg.GEAR_R && abs(v_des) > 0.02 && v_drive < cfg.stuck_v
        desired_ax = max(desired_ax, cfg.unstick_ax);
    end
    desired_ax = max(min(desired_ax, cfg.ax_max), -cfg.ax_max);
    if v_drive > cfg.v_emax                                % SAFETY NET: genuine runaway only (PI now
        desired_ax = -cfg.ax_brake;                       % regulates normal speed; no mid-maneuver stops)
    end
    [steer_fl, steer_fr] = pp_ackermann(delta, L, tw);
end
end

% ===== worst-corner error vs goal footprint (4-corner success metric) ======
function e = corner_err_(x, y, th, gx, gy, gth, ah, bh, hw)
%#codegen
% Max distance [m] between the ego footprint corners at (x,y,th) and the goal
% footprint corners at (gx,gy,gth). Used as the CORRECT good-enough / keep-best
% metric, matching the "4 corners within slot + 0.2 m" success criterion.
cc = pp_rect_corners(x,  y,  th,  ah, bh, hw);
gc = pp_rect_corners(gx, gy, gth, ah, bh, hw);
e = 0.0;
for k = 1:4
    d = hypot(cc(k,1)-gc(k,1), cc(k,2)-gc(k,2));
    if d > e, e = d; end
end
end
