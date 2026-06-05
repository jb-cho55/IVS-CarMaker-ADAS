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
    if d_goal < cfg.goal_reach || (sprog <= 0 && d_goal < cfg.goal_near)   % overshoot only counts NEAR the goal
        v_des = 0;
        if abs(v) < cfg.v_stop, done = true; delta = 0; end
    end
end
end
