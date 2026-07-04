function d = pp_angdiff(a, b)
%#codegen
% PP_ANGDIFF  Smallest signed angle a-b wrapped to [-pi, pi].
d = mod(a - b + pi, 2*pi) - pi;
end
