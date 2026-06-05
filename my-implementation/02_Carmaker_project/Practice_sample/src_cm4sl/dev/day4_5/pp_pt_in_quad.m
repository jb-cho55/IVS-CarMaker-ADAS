function tf = pp_pt_in_quad(px, py, c)
%#codegen
% PP_PT_IN_QUAD  True if point (px,py) lies inside the convex quad c (4-by-2).
%   Winding-agnostic: inside iff the point is on the same side of all 4 edges.
s = zeros(1, 4);
for k = 1:4
    k2 = mod(k, 4) + 1;
    ex = c(k2,1) - c(k,1);
    ey = c(k2,2) - c(k,2);
    s(k) = ex*(py - c(k,2)) - ey*(px - c(k,1));
end
tf = all(s >= -1e-9) || all(s <= 1e-9);
end
