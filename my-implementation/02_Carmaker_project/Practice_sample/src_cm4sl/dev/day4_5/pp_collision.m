function tf = pp_collision(map, x, y, yaw, ahead, behind, halfw, b, n)
%#codegen
% PP_COLLISION  True if the oriented footprint at pose (x,y,yaw) overlaps any
%   occupied cell (map > 0). Pose is the REAR-AXLE reference; the footprint
%   spans [-behind,+ahead] x [-halfw,+halfw]. Scans the footprint bounding box.
c = pp_rect_corners(x, y, yaw, ahead, behind, halfw);
ixv = zeros(1, 4); iyv = zeros(1, 4);
for k = 1:4
    [ixv(k), iyv(k)] = pp_world2grid(c(k,1), c(k,2), b, n);
end
imin = max(min(ixv), 1); imax = min(max(ixv), n);
jmin = max(min(iyv), 1); jmax = min(max(iyv), n);
tf = false;
for ix = imin:imax
    for iy = jmin:jmax
        if map(iy, ix) > 0
            [cx, cy] = pp_grid2world(ix, iy, b, n);
            if pp_pt_in_quad(cx, cy, c)
                tf = true;
                return;
            end
        end
    end
end
end
