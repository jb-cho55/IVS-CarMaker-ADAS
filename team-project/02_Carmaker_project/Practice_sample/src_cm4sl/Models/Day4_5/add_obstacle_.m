function y = add_obstacle_(map, traffic_info, traffic_size)
%ADD_OBSTACLE_ Build a half-width-inflated binary occupancy map.
%
%   y = add_obstacle_(map, traffic_info, traffic_size)
%
%   traffic_info is CarMaker Traffic.StartPos [x;y;yaw] repeated for T00..T20.
%   traffic_size is [width; length].
%   The ego/path state is rear-bumper based, but CarMaker traffic StartPos is
%   the traffic-object reference point near the box center, not the rear bumper.
%
%#codegen

u = map(1, 1) * 0 + traffic_info(1) * 0 + traffic_size(1) * 0;

persistent cached_occ cached_init
if isempty(cached_init)
    cached_occ = double(add_obstacle_local(uint8(map), traffic_info, traffic_size));
    cached_init = true;
end

y = cached_occ + u;
end

function occ = add_obstacle_local(base_map, traffic_info, traffic_size)
%#codegen
c = map_const_local();
N = c.N;
res = c.RES;
x_min = c.X_MIN;
y_max = c.Y_MAX;

veh_w = traffic_size(1);
veh_l = traffic_size(2);

static_map = base_map;

num_traffic = int32(floor(double(numel(traffic_info)) / 3.0));
if num_traffic > int32(21)
    num_traffic = int32(21);
end

for i = 1:num_traffic
    idx = (i - 1) * 3;
    ref_x = traffic_info(idx + 1);
    ref_y = traffic_info(idx + 2);
    yaw    = traffic_info(idx + 3);

    if abs(ref_x) < 1.0e-6 && abs(ref_y) < 1.0e-6 && abs(yaw) < 1.0e-6
        continue;
    end

    c_yaw = cos(yaw);
    s_yaw = sin(yaw);

    half_w = veh_w * 0.5;
    pad = c.RES;
    x0 = c.TRAFFIC_BOX_X_OFFSET - 0.5 * veh_l;
    x1 = c.TRAFFIC_BOX_X_OFFSET + 0.5 * veh_l;
    xs = [x0, x0, x1, x1];
    ys = [-half_w, half_w, half_w, -half_w];
    wx_min =  1.0e18; wx_max = -1.0e18;
    wy_min =  1.0e18; wy_max = -1.0e18;
    for ci = 1:4
        wx_c = ref_x + xs(ci) * c_yaw - ys(ci) * s_yaw;
        wy_c = ref_y + xs(ci) * s_yaw + ys(ci) * c_yaw;
        if wx_c < wx_min; wx_min = wx_c; end
        if wx_c > wx_max; wx_max = wx_c; end
        if wy_c < wy_min; wy_min = wy_c; end
        if wy_c > wy_max; wy_max = wy_c; end
    end
    wx_min = wx_min - pad; wx_max = wx_max + pad;
    wy_min = wy_min - pad; wy_max = wy_max + pad;

    col_min = int32(floor((wx_min - x_min) / res)) + int32(1);
    col_max = int32(floor((wx_max - x_min) / res)) + int32(1);
    row_min = int32(floor((y_max - wy_max) / res)) + int32(1);
    row_max = int32(floor((y_max - wy_min) / res)) + int32(1);

    col_min = max(int32(1), col_min - int32(1));
    col_max = min(N,        col_max + int32(1));
    row_min = max(int32(1), row_min - int32(1));
    row_max = min(N,        row_max + int32(1));

    for row = row_min:row_max
        wy = y_max - (double(row) - 0.5) * res;
        for col = col_min:col_max
            wx = x_min + (double(col) - 0.5) * res;
            dx = wx - ref_x;
            dy = wy - ref_y;
            lx =  c_yaw * dx + s_yaw * dy;
            ly = -s_yaw * dx + c_yaw * dy;
            if lx >= x0 && lx <= x1 && abs(ly) <= half_w
                static_map(row, col) = uint8(1);
            end
        end
    end
end

% Adjacent parked cars often leave a mathematically free but physically unsafe
% slot gap. Treat those narrow side-by-side gaps as blocked so the planner does
% not try to squeeze an EV6 through spaces such as T00-T12 (~3.3 m clear).
static_map = close_side_by_side_gaps(static_map, traffic_info, traffic_size);

% Keep obstacle inflation modest; ego width is handled by the planner's
% swept-footprint collision checker.
inflate_r = c.CENTERLINE_SAFETY_BUFFER;
occ = inflate_binary_map(static_map, inflate_r);
end

function occ_map = close_side_by_side_gaps(occ_map, traffic_info, traffic_size)
%#codegen
c = map_const_local();
num_traffic = int32(floor(double(numel(traffic_info)) / 3.0));
if num_traffic > int32(21)
    num_traffic = int32(21);
end

for i = int32(1):num_traffic-int32(1)
    idx_i = (i - 1) * 3;
    xi = traffic_info(idx_i + 1);
    yi = traffic_info(idx_i + 2);
    yawi = traffic_info(idx_i + 3);
    if abs(xi) < 1.0e-6 && abs(yi) < 1.0e-6 && abs(yawi) < 1.0e-6
        continue;
    end
    [ix0, ix1, iy0, iy1] = vehicle_bbox(xi, yi, yawi, traffic_size);

    for j = i+int32(1):num_traffic
        idx_j = (j - 1) * 3;
        xj = traffic_info(idx_j + 1);
        yj = traffic_info(idx_j + 2);
        yawj = traffic_info(idx_j + 3);
        if abs(xj) < 1.0e-6 && abs(yj) < 1.0e-6 && abs(yawj) < 1.0e-6
            continue;
        end
        if abs(wrap_pi_local(yawi - yawj)) > 0.25
            continue;
        end

        [jx0, jx1, jy0, jy1] = vehicle_bbox(xj, yj, yawj, traffic_size);
        y_overlap = min(iy1, jy1) - max(iy0, jy0);
        if y_overlap < 2.5
            continue;
        end

        gap_x0 = 0.0;
        gap_x1 = 0.0;
        if ix1 < jx0
            gap_x0 = ix1;
            gap_x1 = jx0;
        elseif jx1 < ix0
            gap_x0 = jx1;
            gap_x1 = ix0;
        else
            continue;
        end

        gap_w = gap_x1 - gap_x0;
        if gap_w < 0.40 || gap_w > c.NARROW_GAP_BLOCK_MAX
            continue;
        end

        pad_x = c.NARROW_GAP_PAD_X;
        pad_y = c.NARROW_GAP_PAD_Y;
        x_min = gap_x0 - pad_x;
        x_max = gap_x1 + pad_x;
        y_min = max(iy0, jy0) - pad_y;
        y_max = min(iy1, jy1) + pad_y;
        occ_map = fill_world_rect(occ_map, x_min, x_max, y_min, y_max, c);
    end
end
end

function [wx_min, wx_max, wy_min, wy_max] = vehicle_bbox(ref_x, ref_y, yaw, traffic_size)
%#codegen
c = map_const_local();
veh_w = traffic_size(1);
veh_l = traffic_size(2);
c_yaw = cos(yaw);
s_yaw = sin(yaw);
half_w = veh_w * 0.5;
x0 = c.TRAFFIC_BOX_X_OFFSET - 0.5 * veh_l;
x1 = c.TRAFFIC_BOX_X_OFFSET + 0.5 * veh_l;
xs = [x0, x0, x1, x1];
ys = [-half_w, half_w, half_w, -half_w];
wx_min =  1.0e18; wx_max = -1.0e18;
wy_min =  1.0e18; wy_max = -1.0e18;
for k = 1:4
    wx = ref_x + xs(k) * c_yaw - ys(k) * s_yaw;
    wy = ref_y + xs(k) * s_yaw + ys(k) * c_yaw;
    if wx < wx_min; wx_min = wx; end
    if wx > wx_max; wx_max = wx; end
    if wy < wy_min; wy_min = wy; end
    if wy > wy_max; wy_max = wy; end
end
end

function occ_map = fill_world_rect(occ_map, x_min, x_max, y_min, y_max, c)
%#codegen
col_min = int32(floor((x_min - c.X_MIN) / c.RES)) + int32(1);
col_max = int32(floor((x_max - c.X_MIN) / c.RES)) + int32(1);
row_min = int32(floor((c.Y_MAX - y_max) / c.RES)) + int32(1);
row_max = int32(floor((c.Y_MAX - y_min) / c.RES)) + int32(1);
col_min = max(int32(1), col_min);
col_max = min(c.N, col_max);
row_min = max(int32(1), row_min);
row_max = min(c.N, row_max);
for row = row_min:row_max
    for col = col_min:col_max
        occ_map(row, col) = uint8(1);
    end
end
end

function a = wrap_pi_local(a)
%#codegen
while a > pi;  a = a - 2.0 * pi; end
while a < -pi; a = a + 2.0 * pi; end
end

function inflated = inflate_binary_map(occ_map, radius)
%#codegen
c = map_const_local();
inflated = uint8(occ_map);
rad_cells = int32(ceil(radius / c.RES));
r2 = radius * radius;

for row = int32(1):c.N
    for col = int32(1):c.N
        if occ_map(row, col) > 0
            r0 = max(int32(1), row - rad_cells);
            r1 = min(c.N, row + rad_cells);
            c0 = max(int32(1), col - rad_cells);
            c1 = min(c.N, col + rad_cells);
            for rr = r0:r1
                dy = double(rr - row) * c.RES;
                for cc = c0:c1
                    dx = double(cc - col) * c.RES;
                    if dx * dx + dy * dy <= r2
                        inflated(rr, cc) = uint8(1);
                    end
                end
            end
        end
    end
end
end

function c = map_const_local()
%#codegen
c.N        = int32(200);
c.RES      = 0.5;
c.X_MIN    = 0.0;
c.X_MAX    = 100.0;
c.Y_MIN    = -100.0;
c.Y_MAX    = 0.0;
c.TRUCK_W  = 2.48;
c.TRUCK_L  = 11.5;
c.EGO_W    = 1.9;
c.EGO_L    = 4.7;
c.WHEELBASE = 2.8;
c.TRAFFIC_BOX_X_OFFSET = 0.15;
c.SAFETY_MARGIN = 0.8;
c.CENTERLINE_SAFETY_BUFFER = 0.10;
c.NARROW_GAP_BLOCK_MAX = 4.20;
c.NARROW_GAP_PAD_X = 1.25;
c.NARROW_GAP_PAD_Y = 1.35;
c.CLEAR_MAX = 3.0;
c.W_CLEAR   = 1.2;
c.PARK_BOX_L = 6.0;
c.PARK_BOX_W = 3.0;
c.PARK_TOL   = 0.05;
end
