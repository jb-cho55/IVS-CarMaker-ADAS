function [delta, v_des, done] = pp_align(ax, ay, ath, gx, gy, gth, cfg)
%#codegen
% PP_ALIGN  Precise pose alignment by Reeds-Shepp feedback (rear-axle frame).
%   Recomputes the RS path from the current pose to the goal each step and
%   follows its first segment (feedforward curvature + direction). RS ends
%   EXACTLY at the goal pose, so repeated following converges, using the small
%   corrective arcs/cusps RS provides for any residual lateral/heading offset.
%   Out: delta [rad], v_des [m/s] (signed), done.
done = false;
if hypot(gx-ax, gy-ay) < cfg.align_pos && abs(pp_angdiff(ath, gth)) < cfg.align_yaw
    delta = 0; v_des = 0; done = true;
    return;
end

[T, segLen, ~, ok] = pp_reedsshepp(ax, ay, ath, gx, gy, gth, cfg.Rmin);
if ~ok
    delta = 0; v_des = 0;
    return;
end

% first significant segment
i1 = 1;
if abs(segLen(1)) < 1e-3
    if abs(segLen(2)) >= 1e-3, i1 = 2; else, i1 = 3; end
end
dirn = 1.0; if segLen(i1) < 0, dirn = -1.0; end

% segment curvature -> steering (signed-arc convention: delta = atan(L*kappa),
% valid for forward and reverse, matching pp_rs_sample integration)
if T(i1) == 1
    kappa = 1/cfg.Rmin;
elseif T(i1) == 3
    kappa = -1/cfg.Rmin;
else
    kappa = 0;
end
delta = atan(cfg.wheelbase * kappa);
delta = max(min(delta, cfg.delta_max), -cfg.delta_max);

v_des = dirn * cfg.v_align;     % creep in the first-segment direction
end
