function b = pp_bounds(corners)
%#codegen
% PP_BOUNDS  Axis-aligned bounds of a rectangular map boundary.
%   corners : K-by-2 list of (x,y) corner points (the model's map_boundary,
%             reshaped to K-by-2 before calling).
%   b       : [xmin xmax ymin ymax].
xs = corners(:,1);
ys = corners(:,2);
b = [min(xs), max(xs), min(ys), max(ys)];
end
