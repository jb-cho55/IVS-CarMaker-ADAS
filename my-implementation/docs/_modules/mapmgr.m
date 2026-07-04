function MapCtx = mapmgr(Ego, LANE_XY, LANE_LEN)
%#codegen
tx=Ego(1); ty=Ego(2); nL=size(LANE_XY,3);
cur=1; bestd=inf; sidx=1;
for li=1:nL
  n=LANE_LEN(li); dmin=inf; im=1;
  for k=1:n
    dx=LANE_XY(k,1,li)-tx; dy=LANE_XY(k,2,li)-ty; d=dx*dx+dy*dy;
    if d<dmin; dmin=d; im=k; end
  end
  if dmin<bestd; bestd=dmin; cur=li; sidx=im; end
end
MapCtx=[cur; sidx; sidx/max(1,LANE_LEN(cur)); sqrt(bestd)];
end