function map = pp_fill_rect(map, c, b, n, val)
%#codegen
% PP_FILL_RECT  Rasterize a filled (possibly rotated) rectangle onto the grid.
%   c   : 4-by-2 world corners. b,n : grid definition. val : value to write.
%   Cells whose center falls inside the quad are set to val. map is n-by-n,
%   indexed map(iy, ix) (row = Y index, col = X index).
ixv = zeros(1, 4); iyv = zeros(1, 4);
for k = 1:4
    [ixv(k), iyv(k)] = pp_world2grid(c(k,1), c(k,2), b, n);
end
imin = max(min(ixv), 1); imax = min(max(ixv), n);
jmin = max(min(iyv), 1); jmax = min(max(iyv), n);
for ix = imin:imax
    for iy = jmin:jmax
        [cx, cy] = pp_grid2world(ix, iy, b, n);
        if pp_pt_in_quad(cx, cy, c)
            map(iy, ix) = val;
        end
    end
end
end
