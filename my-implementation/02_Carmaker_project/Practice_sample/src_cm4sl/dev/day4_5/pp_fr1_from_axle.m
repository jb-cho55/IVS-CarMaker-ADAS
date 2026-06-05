function [xf, yf] = pp_fr1_from_axle(xa, ya, yaw, d_r)
%#codegen
% PP_FR1_FROM_AXLE  Fr1 (rear-bumper) position from rear-axle pose.
xf = xa - d_r*cos(yaw);
yf = ya - d_r*sin(yaw);
end
