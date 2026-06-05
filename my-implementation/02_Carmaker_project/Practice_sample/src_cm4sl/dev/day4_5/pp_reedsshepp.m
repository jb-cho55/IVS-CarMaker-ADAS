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
