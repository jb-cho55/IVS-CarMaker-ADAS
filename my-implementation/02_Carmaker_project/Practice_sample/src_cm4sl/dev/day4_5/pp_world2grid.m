function [ix, iy] = pp_world2grid(x, y, b, n)
%#codegen
% PP_WORLD2GRID  World (x,y) -> 1-based grid indices (ix along X/col, iy along Y/row).
%   b = [xmin xmax ymin ymax], n = cells per side. Indices clamped to [1,n].
resx = (b(2) - b(1)) / n;
resy = (b(4) - b(3)) / n;
ix = floor((x - b(1)) / resx) + 1;
iy = floor((y - b(3)) / resy) + 1;
ix = min(max(ix, 1), n);
iy = min(max(iy, 1), n);
end
