function [dL, dR] = pp_ackermann(delta, L, tw)
%#codegen
% PP_ACKERMANN  Left/right front road-wheel angles from a single bicycle steer.
%   delta : bicycle (center) steer angle [rad]; L : wheelbase; tw : front track.
%   Inner wheel steers more than the outer (true Ackermann geometry).
if abs(delta) < 1e-4
    dL = 0.0; dR = 0.0;
    return;
end
R  = L / tan(delta);            % signed turn radius at rear-axle center (+ = left)
dL = atan(L / (R - tw/2));      % left  front wheel
dR = atan(L / (R + tw/2));      % right front wheel
end
