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
