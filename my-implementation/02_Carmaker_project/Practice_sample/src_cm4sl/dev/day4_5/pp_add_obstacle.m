function y = pp_add_obstacle(map, traffic_info, traffic_size) %#ok<INUSD>
%#codegen
% PP_ADD_OBSTACLE  Mark static parking obstacles onto the base map (M3 logic).
%   y = map with each obstacle footprint rasterized as occupied (1).
%
%   Obstacles are static & known, so they come from pp_obstacles() (rear-bumper
%   reference), inflated by cfg.margin on every side. The model's built-in
%   traffic reader (T01..T07) does NOT match the final lot's 21 cars, so
%   traffic_info / traffic_size are intentionally unused (kept for block
%   signature compatibility). Wire live traffic here only if the reader is
%   extended to all lot obstacles.
%
%   PERFORMANCE: the result is STATIC (cars never move; base map is the cached
%   constant from pp_generate_map), so it is computed ONCE and cached. Codegen
%   rule: the persistent is assigned inside the `if isempty(...)` guard BEFORE it
%   is read. Cache clears at sim start and by `clear functions`.
persistent FULL
if isempty(FULL)
    cfg = pp_cfg();
    n = cfg.n;
    b = cfg.bounds;
    yy = map;
    OBST = pp_obstacles();
    ahead  = cfg.obs_L + cfg.margin;
    behind = cfg.margin;
    halfw  = cfg.obs_W/2 + cfg.margin;
    for k = 1:size(OBST, 1)
        if k == cfg.holdout, continue; end          % keep the target slot free
        c = pp_rect_corners(OBST(k,1), OBST(k,2), OBST(k,3), ahead, behind, halfw);
        yy = pp_fill_rect(yy, c, b, n, 1);
    end
    FULL = yy;
end
y = FULL;
end
