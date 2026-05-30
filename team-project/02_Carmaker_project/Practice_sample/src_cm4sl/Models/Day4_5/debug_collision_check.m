function debug_collision_check()
close all force;
this_dir = fileparts(mfilename('fullpath'));
addpath(this_dir);

start_point = [5.5, -36.5];
finish_point = [17.0, -7.0];
ego_yaw = pi;
goal_yaw = -pi/2;

map_boundary = [
    4.0,  -4.0
    4.0, -46.8
   48.0,  -4.0
   48.0, -46.8
];

traffic_size = [1.97; 4.47];
traffic_info = [
     7.3; -28.7; -pi/2
    12.8;  -6.8; -pi/2
    21.3;  -6.6; -pi/2
    30.0;  -6.5; -pi/2
    41.7;  -6.3; -pi/2
     6.9;  -6.9; -pi/2
     7.0; -21.8;  pi/2
    18.6; -21.8;  pi/2
    24.3; -21.9;  pi/2
    36.0; -21.9;  pi/2
    38.8; -21.8;  pi/2
    41.8; -21.8;  pi/2
    12.6; -28.8; -pi/2
    24.3; -28.9; -pi/2
    41.6; -28.9; -pi/2
     9.9; -44.4;  pi/2
    21.4; -44.3;  pi/2
    24.5; -44.4;  pi/2
    36.0; -44.4;  pi/2
    41.8; -44.5;  pi/2
    44.6; -44.4;  pi/2
];

base_map = generate_map_(map_boundary, traffic_info, traffic_size);
occ_map = add_obstacle_(base_map, traffic_info, traffic_size);

fprintf('start occupied samples: %d\n', count_centerline_hits(occ_map, start_point(1), start_point(2), ego_yaw));
fprintf('goal occupied samples: %d\n', count_centerline_hits(occ_map, finish_point(1), finish_point(2), goal_yaw));
end

function hits = count_centerline_hits(occ_map, rear_x, rear_y, yaw)
hits = 0;
for i = 1:12
    s = 4.7 * (i - 1) / 11;
    x = rear_x + s * cos(yaw);
    y = rear_y + s * sin(yaw);
    col = floor((x - 0.0) / 0.5) + 1;
    row = floor((0.0 - y) / 0.5) + 1;
    is_hit = row < 1 || row > 200 || col < 1 || col > 200 || occ_map(row, col) > 0;
    if is_hit
        hits = hits + 1;
    end
    fprintf('  s=%4.2f x=%6.2f y=%6.2f row=%3d col=%3d occ=%d\n', s, x, y, row, col, is_hit);
end
end
