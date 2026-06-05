function [x, y] = pp_grid2world(ix, iy, b, n)
%#codegen
% PP_GRID2WORLD  1-based grid indices -> world coordinates of the CELL CENTER.
%   b = [xmin xmax ymin ymax], n = cells per side.
resx = (b(2) - b(1)) / n;
resy = (b(4) - b(3)) / n;
x = b(1) + (ix - 0.5) * resx;
y = b(3) + (iy - 0.5) * resy;
end
