function c = pp_rect_corners(x, y, yaw, ahead, behind, halfw)
%#codegen
% PP_RECT_CORNERS  World corners (4-by-2) of an oriented rectangle.
%   Local frame: X forward, Y left. Rectangle spans X in [-behind, +ahead]
%   and Y in [-halfw, +halfw], placed at pose (x, y, yaw).
%   Corner order: front-left, front-right, rear-right, rear-left (clockwise).
co = cos(yaw); si = sin(yaw);
lx = [ahead,  ahead, -behind, -behind];
ly = [halfw, -halfw, -halfw,   halfw];
c = zeros(4, 2);
for k = 1:4
    c(k,1) = x + co*lx(k) - si*ly(k);
    c(k,2) = y + si*lx(k) + co*ly(k);
end
end
