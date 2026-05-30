function plot_current_parking_path_matlab(mode)
%PLOT_CURRENT_PARKING_PATH_MATLAB Run the current Day4_5 MATLAB functions.
%
% This uses the same external function files currently copied into Day4_5:
% generate_map_.m, add_obstacle_.m, and Parking.m.

if nargin < 1
    mode = 'entrance';
end

close all force;
clear Parking generate_map_ add_obstacle_;

this_dir = fileparts(mfilename('fullpath'));
addpath(this_dir);

finish_point = [17.0, -7.0];
goal_yaw = -pi/2;
case_name = 'staging';
out_name = 'current_parking_path_from_matlab.png';

switch lower(char(mode))
    case 'entrance'
        start_point = [5.5, -36.5];
        ego_yaw = pi;
        case_name = 'entrance';
        out_name = 'current_parking_path_from_entrance_matlab.png';
    otherwise
        start_point = [17.0, -11.5];
        ego_yaw = -pi/2;
end

ego_v = 0.0;

map_boundary = [
    4.0,  -4.0
    4.0, -46.8
   48.0,  -4.0
   48.0, -46.8
];

traffic_size = [1.97; 4.47];  % IPG_CompanyCar_2018_Blue [width; length]
traffic_ref_x_offset = 0.15;  % CarMaker Traffic.Basics.Offset x
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

[desired_ax, steer_fl, steer_fr, path_x, path_y, path_len, selector_ctrl, vc_gas, vc_brake] = ...
    Parking(start_point(1), start_point(2), ego_yaw, ego_v, ...
    start_point, finish_point, goal_yaw, occ_map);

path_len = double(path_len);
valid_idx = 1:max(1, min(path_len, numel(path_x)));

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1100 900]);
ax = axes(fig);
hold(ax, 'on');

imagesc(ax, [0 100], [-100 0], flipud(occ_map));
set(ax, 'YDir', 'normal');
colormap(ax, [0.96 0.97 0.94; 0.75 0.78 0.74]);

draw_boundary(ax, map_boundary);
draw_traffic(ax, traffic_info, traffic_size, traffic_ref_x_offset);
draw_centerline(ax, start_point(1), start_point(2), ego_yaw, 'CENTERLINE CHECK FROM REAR START');
draw_centerline(ax, finish_point(1), finish_point(2), goal_yaw, 'CENTERLINE CHECK FROM REAR GOAL');
draw_goal_replan_circle(ax, finish_point(1), finish_point(2), 6.5);

if path_len >= 2
    ego_plot_width = 1.9 + 2.0 * 0.25;
    draw_path_tube(ax, path_x(valid_idx), path_y(valid_idx), ego_plot_width);
    plot(ax, path_x(valid_idx), path_y(valid_idx), 'r-', 'LineWidth', 3);
else
    text(ax, 8, -34, 'NO VALID PATH FROM Parking.m', 'Color', 'r', ...
        'FontWeight', 'bold', 'FontSize', 12);
end

plot(ax, start_point(1), start_point(2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 9);
text(ax, start_point(1)+0.7, start_point(2), 'REAR START', 'Color', [0 0.35 0]);
plot(ax, finish_point(1), finish_point(2), 'yo', 'MarkerFaceColor', 'y', 'MarkerSize', 9);
text(ax, finish_point(1)+0.7, finish_point(2), 'REAR GOAL', 'Color', [0.45 0.33 0]);

axis(ax, 'equal');
xlim(ax, [0 52]);
ylim(ax, [-50 0]);
grid(ax, 'on');
xlabel(ax, 'x [m]');
ylabel(ax, 'y [m]');
title(ax, sprintf('%s start | RS-first + Hybrid A* + smoothed fallback | len=%d | gas=%.2f brake=%.2f sel=%.0f', ...
    case_name, path_len, vc_gas, vc_brake, selector_ctrl));

out_png = fullfile(this_dir, out_name);
saveas(fig, out_png);
close(fig);

fprintf('path_len=%d\n', path_len);
fprintf('desired_ax=%.6f steer_fl=%.6f steer_fr=%.6f selector=%.1f gas=%.6f brake=%.6f\n', ...
    desired_ax, steer_fl, steer_fr, selector_ctrl, vc_gas, vc_brake);
fprintf('%s\n', out_png);
end

function draw_path_tube(ax, x, y, ego_width)
x = x(:);
y = y(:);
n = numel(x);
if n < 2
    return;
end

nx = zeros(n, 1);
ny = zeros(n, 1);
for i = 1:n
    if i == 1
        tx = x(2) - x(1);
        ty = y(2) - y(1);
    elseif i == n
        tx = x(n) - x(n-1);
        ty = y(n) - y(n-1);
    else
        tx = x(i+1) - x(i-1);
        ty = y(i+1) - y(i-1);
    end
    L = hypot(tx, ty);
    if L < 1.0e-6
        nx(i) = 0;
        ny(i) = 0;
    else
        nx(i) = -ty / L;
        ny(i) =  tx / L;
    end
end

half_w = ego_width * 0.5;
left_x = x + half_w * nx;
left_y = y + half_w * ny;
right_x = flipud(x - half_w * nx);
right_y = flipud(y - half_w * ny);

patch(ax, [left_x; right_x], [left_y; right_y], [1.0 0.20 0.10], ...
    'FaceAlpha', 0.18, 'EdgeColor', 'none');
end

function draw_goal_replan_circle(ax, gx, gy, radius)
theta = linspace(0, 2*pi, 160);
plot(ax, gx + radius*cos(theta), gy + radius*sin(theta), ...
    '-', 'Color', [0.95 0.45 0.05], 'LineWidth', 1.4);
end

function draw_boundary(ax, map_boundary)
pts = sort_poly(map_boundary);
pts = [pts; pts(1,:)];
plot(ax, pts(:,1), pts(:,2), 'k-', 'LineWidth', 2);
end

function pts = sort_poly(pts)
c = mean(pts, 1);
[~, idx] = sort(atan2(pts(:,2)-c(2), pts(:,1)-c(1)));
pts = pts(idx, :);
end

function draw_traffic(ax, traffic_info, traffic_size, traffic_ref_x_offset)
w = traffic_size(1);
l = traffic_size(2);
for i = 1:floor(numel(traffic_info)/3)
    idx = (i-1)*3;
    x = traffic_info(idx+1);
    y = traffic_info(idx+2);
    yaw = traffic_info(idx+3);
    draw_vehicle(ax, x, y, yaw, l, w, traffic_ref_x_offset, sprintf('T%02d', i-1));
end
end

function draw_inflation_circles(ax, traffic_info, traffic_size)
w = traffic_size(1);
l = traffic_size(2);
r = sqrt((l/6)^2 + (w/2)^2) + 0.40;
circle_x = [l/6, l/2, 5*l/6];
theta = linspace(0, 2*pi, 64);
for i = 1:floor(numel(traffic_info)/3)
    idx = (i-1)*3;
    x = traffic_info(idx+1);
    y = traffic_info(idx+2);
    yaw = traffic_info(idx+3);
    for j = 1:3
        cx = x + circle_x(j) * cos(yaw);
        cy = y + circle_x(j) * sin(yaw);
        plot(ax, cx + r*cos(theta), cy + r*sin(theta), ...
            '-', 'Color', [0.38 0.44 0.38], 'LineWidth', 0.75);
    end
end
end

function draw_centerline(ax, rear_x, rear_y, yaw, label)
l = 4.7;
s = linspace(0, l, 12);
x = rear_x + s * cos(yaw);
y = rear_y + s * sin(yaw);
plot(ax, x, y, '--', 'Color', [0.9 0.25 0.02], 'LineWidth', 1.2);
plot(ax, x, y, '.', 'Color', [0.9 0.25 0.02], 'MarkerSize', 8);
text(ax, rear_x+0.5, rear_y-0.8, label, 'Color', [0.7 0.22 0.02], 'FontSize', 7);
end

function draw_vehicle(ax, ref_x, ref_y, yaw, l, w, traffic_ref_x_offset, label)
c = cos(yaw);
s = sin(yaw);
half_w = w/2;
x0 = traffic_ref_x_offset - 0.5 * l;
x1 = traffic_ref_x_offset + 0.5 * l;
local = [x0 -half_w; x0 half_w; x1 half_w; x1 -half_w];
world = zeros(4,2);
for k = 1:4
    lx = local(k,1);
    ly = local(k,2);
    world(k,1) = ref_x + lx*c - ly*s;
    world(k,2) = ref_y + lx*s + ly*c;
end
patch(ax, world(:,1), world(:,2), [0.25 0.55 0.85], ...
    'EdgeColor', [0.05 0.22 0.4], 'FaceAlpha', 0.85);
plot(ax, ref_x, ref_y, '.', 'Color', [0.01 0.08 0.16], 'MarkerSize', 9);
text(ax, ref_x, ref_y, label, 'Color', [0.02 0.12 0.25], 'FontSize', 7);
end
