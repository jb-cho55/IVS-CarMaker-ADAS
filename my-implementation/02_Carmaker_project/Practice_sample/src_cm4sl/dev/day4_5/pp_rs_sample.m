function [sx, sy, sth, sdir, ns] = pp_rs_sample(x0, y0, th0, T, segLen, Rmin, ds, MAXP)
%#codegen
% PP_RS_SAMPLE  Sample a Reeds-Shepp path into poses for collision checks/output.
%   T(1x3) types (1=L,2=S,3=R), segLen(1x3) signed arc lengths [m].
%   Returns poses sx,sy,sth (MAXP x1), per-point direction sdir (+1/-1), ns count.
sx = zeros(MAXP,1); sy = zeros(MAXP,1); sth = zeros(MAXP,1); sdir = zeros(MAXP,1);
x = x0; y = y0; th = th0;
k = 1; sx(1) = x; sy(1) = y; sth(1) = th; sdir(1) = sign_nz(segLen(1));

for i = 1:3
    Li = segLen(i);
    if abs(Li) < 1e-9, continue; end
    dirn = sign_nz(Li);
    if T(i) == 1
        kappa =  1/Rmin;
    elseif T(i) == 3
        kappa = -1/Rmin;
    else
        kappa = 0;
    end
    dist  = abs(Li);
    steps = max(1, ceil(dist/ds));
    step  = dist/steps;
    for j = 1:steps
        if k >= MAXP, break; end
        sgn = dirn*step;
        if kappa == 0
            x = x + sgn*cos(th);
            y = y + sgn*sin(th);
        else
            th2 = th + sgn*kappa;
            x = x + (sin(th2) - sin(th))/kappa;
            y = y - (cos(th2) - cos(th))/kappa;
            th = th2;
        end
        k = k + 1;
        sx(k) = x; sy(k) = y; sth(k) = th; sdir(k) = dirn;
    end
end
ns = k;
end

function s = sign_nz(v)
%#codegen
if v < 0, s = -1.0; else, s = 1.0; end
end
