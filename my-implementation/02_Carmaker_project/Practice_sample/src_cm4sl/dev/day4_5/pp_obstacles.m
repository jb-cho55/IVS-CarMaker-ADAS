function OBST = pp_obstacles()
%#codegen
% PP_OBSTACLES  Static parking-lot obstacle table for the mission.
%   Returns K-by-3 array, each row [x y yaw] = obstacle REAR-BUMPER center [m]
%   and heading [rad] (same convention as ego Car.Fr1).
%
%   Values below are the day7_final lot (T00..T20, IPG_CompanyCar_2018_Blue).
%   *** Replace/confirm with the exact Mission-4 obstacle coordinates. ***
%   Footprint (length cfg.obs_L forward, width cfg.obs_W) is applied in
%   pp_add_obstacle; do NOT pre-offset here.
H = pi/2;
OBST = [ ...
    7.3  -28.7  -H;   % T00
   12.8   -6.8  -H;   % T01
   21.3   -6.6  -H;   % T02
   30.0   -6.5  -H;   % T03
   41.7   -6.3  -H;   % T04
    6.9   -6.9  -H;   % T05
    7.0  -21.8   H;   % T06
   18.6  -21.8   H;   % T07
   24.3  -21.9   H;   % T08
   36.0  -21.9   H;   % T09
   38.8  -21.8   H;   % T10
   41.8  -21.8   H;   % T11
   12.6  -28.8  -H;   % T12
   24.3  -28.9  -H;   % T13
   41.6  -28.9  -H;   % T14
    9.9  -44.4   H;   % T15
   21.4  -44.3   H;   % T16
   24.5  -44.4   H;   % T17
   36.0  -44.4   H;   % T18
   41.8  -44.5   H;   % T19
   44.6  -44.4   H];  % T20
end
