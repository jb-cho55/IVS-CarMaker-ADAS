function [xa, ya] = pp_axle_from_fr1(x, y, yaw, d_r)
%#codegen
% PP_AXLE_FROM_FR1  Rear-axle position from Fr1 (rear-bumper) pose.
%   Rear axle sits d_r ahead of Fr1 along the heading.
xa = x + d_r*cos(yaw);
ya = y + d_r*sin(yaw);
end
