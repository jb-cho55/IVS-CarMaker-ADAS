function [ax,sfl,sfr,gear] = parking_wrap(Ego, Mode, goal, occ)
%#codegen
persistent armed; if isempty(armed); armed=false; end
if Mode(5)>0.5; armed=true; end
if armed
  [ax,sfl,sfr,gear]=pp_parking(Ego(1),Ego(2),Ego(3),Ego(4),[0;0],[goal(1);goal(2)],goal(3),occ);
else
  ax=0.0; sfl=0.0; sfr=0.0; gear=-9.0;
end
end