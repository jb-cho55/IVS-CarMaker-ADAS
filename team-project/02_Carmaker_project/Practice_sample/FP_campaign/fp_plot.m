function fp_plot(Tnum, ax, ay, px, py, gx, gy, gdeg, om, outpath, metrics)
%FP_PLOT  Save a campaign result figure: obstacles + planned + actual + goal.
%   Tnum            : test index (for title/filename)
%   ax,ay           : ACTUAL rear-bumper path (from erg Car_Fr1_tx/ty). [] if none.
%   px,py           : PLANNED rear-bumper path. [] if none.
%   gx,gy,gdeg      : goal rear-bumper pose (deg).
%   om              : 200x200 occupancy grid (1=occupied).
%   outpath         : PNG path to save.
%   metrics         : struct with .err .yawerr .nrev (optional, [] to skip).
b = [-5 48 -45.5 6]; n = 200;            % planner bounds / grid (pp_cfg)
resx = (b(2)-b(1))/n; resy = (b(4)-b(3))/n;
d_r = 0.95; veh_L = 4.68; veh_W = 1.88;  % Kia_EV6 (pp_cfg)

f = figure('Visible','off','Position',[80 80 1100 760]); hold on; axis equal; box on;

% --- obstacles (occupied cells -> world cell centers) ---
[iy,ix] = find(om > 0);
if ~isempty(ix)
    wx = b(1) + (ix-0.5)*resx; wy = b(3) + (iy-0.5)*resy;
    plot(wx, wy, 's', 'Color',[.55 .55 .58], 'MarkerSize',2.5, ...
         'MarkerFaceColor',[.55 .55 .58], 'HandleVisibility','off');
end

h = []; lab = {};
% --- planned path (rear-bumper) ---
if ~isempty(px)
    hp = plot(px, py, 'b--', 'LineWidth',1.6); h(end+1)=hp; lab{end+1}='planned';
end
% --- actual path (rear-bumper) ---
if ~isempty(ax)
    ha = plot(ax, ay, 'r-', 'LineWidth',2.0); h(end+1)=ha; lab{end+1}='actual';
    plot(ax(1),  ay(1),  'go','MarkerSize',9,'MarkerFaceColor','g','HandleVisibility','off');
    plot(ax(end),ay(end),'rx','MarkerSize',11,'LineWidth',2.2,'HandleVisibility','off');
end

% --- goal pose: rear-bumper marker + heading + vehicle footprint slot ---
gth = gdeg*pi/180; co = cos(gth); si = sin(gth);
plot(gx, gy, 'p', 'Color',[1 .5 0], 'MarkerSize',17, 'MarkerFaceColor',[1 .5 0], 'HandleVisibility','off');
quiver(gx, gy, 2.5*co, 2.5*si, 0, 'Color',[1 .5 0], 'LineWidth',2.2, 'MaxHeadSize',2, 'HandleVisibility','off');
% footprint: from Fr1(rear bumper) forward veh_L, lateral +-veh_W/2
lx = [0 veh_L veh_L 0 0]; ly = [veh_W/2 veh_W/2 -veh_W/2 -veh_W/2 veh_W/2];
fx = gx + co*lx - si*ly; fy = gy + si*lx + co*ly;
hg = plot(fx, fy, '-', 'Color',[1 .5 0], 'LineWidth',1.4); h(end+1)=hg; lab{end+1}='goal slot';

xlabel('X [m]'); ylabel('Y [m]'); grid on;
xlim([b(1) b(2)]); ylim([b(3) b(4)]);
ttl = sprintf('T%02d  goal(%.2f, %.2f, %.0f deg)', Tnum, gx, gy, gdeg);
if ~isempty(metrics)
    ttl = sprintf('%s\\newlineerr=%.3f m   yaw=%.2f deg   rev=%d', ttl, metrics.err, metrics.yawerr, metrics.nrev);
end
title(ttl, 'Interpreter','tex');
if ~isempty(h), legend(h, lab, 'Location','northeastoutside'); end

saveas(f, outpath);
close(f);
end
