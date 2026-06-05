function mapMatrix = pp_generate_map(map_boundary, traffic_info, traffic_size) %#ok<INUSD>
%#codegen
% PP_GENERATE_MAP  Base occupancy grid for the parking area (M2 logic).
%   Output mapMatrix is n-by-n, indexed (iy, ix): 0 = free, 1 = occupied.
%
%   Drivable space = the parking-lot rectangle (cfg.lot) UNION the entrance-road
%   corridor (route5 centerline +- halfw, from pp_entrance_path), each inset by
%   cfg.edge_margin. Everything else inside cfg.bounds is OCCUPIED, so the planner
%   cannot cut across non-road area. Static parked cars are stamped afterwards by
%   pp_add_obstacle.
%
%   PERFORMANCE: the map is STATIC, so it is computed ONCE and cached in a
%   persistent (this block runs every sim step; the corridor distance-carve must
%   not run per-step). Codegen rule: the persistent is assigned inside the
%   `if isempty(...)` guard BEFORE it is read. Cache clears at sim start and by
%   `clear functions` after editing cfg / pp_entrance_path.
%
%   map_boundary / traffic_info / traffic_size are accepted only for MATLAB
%   Function block signature compatibility (intentionally unused here).
persistent BASE
if isempty(BASE)
    cfg = pp_cfg();
    n = cfg.n; b = cfg.bounds; em = cfg.edge_margin;
    M = ones(n, n);                                 % default OCCUPIED; carve drivable area free

    % --- carve the parking-lot rectangle FREE (inset by the edge safety margin) -
    lot = cfg.lot;
    [lx0, ly0] = pp_world2grid(lot(1)+em, lot(3)+em, b, n);
    [lx1, ly1] = pp_world2grid(lot(2)-em, lot(4)-em, b, n);
    for ix = min(lx0,lx1):max(lx0,lx1)
        for iy = min(ly0,ly1):max(ly0,ly1)
            M(iy, ix) = 0;
        end
    end

    % --- carve the entrance-road corridor FREE (route5 centerline +- halfw) ----
    [WP, nW, halfw] = pp_entrance_path();
    halfw = max(halfw - em, 0.10);                  % inset corridor walls by the safety margin
    if nW >= 2
        xmnw = min(WP(:,1))-halfw; xmxw = max(WP(:,1))+halfw;
        ymnw = min(WP(:,2))-halfw; ymxw = max(WP(:,2))+halfw;
        [cx0, cy0] = pp_world2grid(xmnw, ymnw, b, n);
        [cx1, cy1] = pp_world2grid(xmxw, ymxw, b, n);
        for ix = min(cx0,cx1):max(cx0,cx1)
            for iy = min(cy0,cy1):max(cy0,cy1)
                [wx, wy] = pp_grid2world(ix, iy, b, n);
                dmin = inf;
                for s = 1:nW-1
                    d = seg_dist_(wx, wy, WP(s,1), WP(s,2), WP(s+1,1), WP(s+1,2));
                    if d < dmin, dmin = d; end
                end
                if dmin <= halfw, M(iy, ix) = 0; end
            end
        end
    end

    % --- occupied border ring (keeps the search bounded) ----------------------
    w = cfg.wall_cells;
    if w > 0
        M(1:w, :)     = 1;
        M(n-w+1:n, :) = 1;
        M(:, 1:w)     = 1;
        M(:, n-w+1:n) = 1;
    end

    BASE = M;
end
mapMatrix = BASE;
end

% ===== distance from point (px,py) to segment (ax,ay)-(bx,by) ==============
function d = seg_dist_(px, py, ax, ay, bx, by)
%#codegen
vx = bx-ax; vy = by-ay;
wx = px-ax; wy = py-ay;
vv = vx*vx + vy*vy;
if vv < 1e-9
    d = hypot(px-ax, py-ay); return;
end
t = (wx*vx + wy*vy) / vv;
if t < 0, t = 0; elseif t > 1, t = 1; end
qx = ax + t*vx; qy = ay + t*vy;
d = hypot(px-qx, py-qy);
end
