function R = cl_metrics(x,y,yaw,T,st,stT,gx,gy,gdeg)
%CL_METRICS  Parking result metrics from a CarMaker co-sim run (clone FP).
%  Inputs: ego rear-bumper traj x,y[m] yaw[rad] at times T; status st at stT;
%          goal gx,gy[m] gdeg[deg] (rear-bumper pose).
%  Output R: parked pose (px,py,pyaw_deg), pos/yaw error, reversals, verdict,
%            entry pose, closest approach, parking-phase indices.
dt = T(2)-T(1);
N  = numel(x);

% --- map status (its own time base) onto traj indices ---
stTraj = interp1(stT, double(st), T, 'previous', 'extrap');

% --- parking phase = status >= 3 ---
pk = find(stTraj >= 3);
R.has_park = ~isempty(pk);
if R.has_park
    k0 = pk(1);            % parking trigger
    k1 = pk(end);          % parking end (give-up OR settled-at-end)
else
    k0 = 1; k1 = N;
end
R.k0 = k0; R.k1 = k1;
R.t_entry = T(k0); R.t_end = T(k1);
R.entry = [x(k0) y(k0) rad2deg(yaw(k0))];

% --- parked pose = pose at end of active parking phase ---
R.px = x(k1); R.py = y(k1); R.pyaw = rad2deg(yaw(k1));
R.perr = hypot(R.px-gx, R.py-gy);
R.yerr = mod(R.pyaw - gdeg + 180, 360) - 180;

% --- closest approach to goal over whole run ---
d2g = hypot(x-gx, y-gy);
[R.dmin, im] = min(d2g);
R.closest = [x(im) y(im) rad2deg(yaw(im)) T(im)];

% --- gear reversals during parking phase (along-heading velocity sign flips) ---
vx = [0; diff(x)]/dt; vy = [0; diff(y)]/dt; spd = hypot(vx,vy);
seg = k0:k1;
sdir = sign(vx(seg).*cos(yaw(seg)) + vy(seg).*sin(yaw(seg)));
sdir(spd(seg) < 0.05) = 0;           % deadband: ignore near-stationary
sdir = sdir(sdir ~= 0);              % keep only moving samples
R.rev = sum(abs(diff(sdir)) > 1);    % +1<->-1 transitions
R.moving_at_end = mean(spd(max(1,N-2000):N)) > 0.05;

% --- verdict (lenient parking tolerance) ---
R.PASS = (R.perr <= 0.50) && (abs(R.yerr) <= 15);
if ~R.has_park
    R.verdict = 'NO_PARK';
elseif R.PASS
    R.verdict = 'PASS';
elseif R.perr <= 1.0 && abs(R.yerr) <= 25
    R.verdict = 'NEAR';
else
    R.verdict = 'FAIL';
end
end
