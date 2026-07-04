function gap = rect_gap(cA, thA, hlA, hwA, cB, thB, hlB, hwB)
%#codegen
% Minimum gap between two oriented rectangles (vehicle footprints).
%   c* = [x y] center, th* = heading [rad], hl* = half-length (along heading),
%   hw* = half-width. Returns >0 separation [m], <=0 if overlapping (approx
%   negative penetration). Uses SAT for overlap, corner/edge min-dist for gap.
A = rect_corners(cA, thA, hlA, hwA);
B = rect_corners(cB, thB, hlB, hwB);
[ov, pen] = sat_overlap(A, B);
if ov
    gap = -pen;                       % overlapping: negative penetration depth
else
    gap = poly_min_dist(A, B);        % separated: min boundary distance
end
end

function C = rect_corners(c, th, hl, hw)
R = [cos(th) -sin(th); sin(th) cos(th)];
loc = [ hl  hw;  hl -hw; -hl -hw; -hl  hw];
C = (R*loc')' + c;                    % 4x2
end

function [ov, pen] = sat_overlap(A, B)
% Separating Axis Theorem overlap test; pen = min overlap along any axis.
ov = true; pen = inf;
axes = [edge_normals(A); edge_normals(B)];
for i = 1:size(axes,1)
    ax = axes(i,:);
    pA = A*ax'; pB = B*ax';
    o = min(max(pA),max(pB)) - max(min(pA),min(pB));
    if o <= 0, ov = false; pen = 0; return; end
    if o < pen, pen = o; end
end
end

function N = edge_normals(P)
N = zeros(4,2);
for i = 1:4
    j = mod(i,4)+1; e = P(j,:)-P(i,:);
    n = [-e(2) e(1)]; N(i,:) = n/max(norm(n),eps);
end
end

function d = poly_min_dist(A, B)
d = inf;
for i = 1:4
    for j = 1:4
        k = mod(j,4)+1;
        d = min(d, pt_seg(A(i,:), B(j,:), B(k,:)));
        d = min(d, pt_seg(B(i,:), A(j,:), A(k,:)));
    end
end
end

function d = pt_seg(p, a, b)
ab = b-a; t = 0; den = ab*ab';
if den>0, t = max(0,min(1,((p-a)*ab')/den)); end
proj = a + t*ab; d = norm(p-proj);
end
