function Path = pathplan(Ego, Mode, LANE_XY, LANE_LEN, APPROACH)
%#codegen
Np=30; tx=Ego(1); ty=Ego(2); Path=zeros(2*Np,1);
if Mode(4)>0.5
  n=size(APPROACH,1); dmin=inf; im=1;
  for k=1:n; dx=APPROACH(k,1)-tx; dy=APPROACH(k,2)-ty; d=dx*dx+dy*dy; if d<dmin; dmin=d; im=k; end; end
  for j=1:Np; idx=im+(j-1); if idx>n; idx=n; end; Path(2*j-1)=APPROACH(idx,1); Path(2*j)=APPROACH(idx,2); end
else
  nL=size(LANE_XY,3); tl=round(Mode(1)); if tl<1; tl=1; elseif tl>nL; tl=nL; end
  n=LANE_LEN(tl); dmin=inf; im=1;
  for k=1:n; dx=LANE_XY(k,1,tl)-tx; dy=LANE_XY(k,2,tl)-ty; d=dx*dx+dy*dy; if d<dmin; dmin=d; im=k; end; end
  for j=1:Np; idx=mod(im-1+(j-1), n)+1; Path(2*j-1)=LANE_XY(idx,1,tl); Path(2*j)=LANE_XY(idx,2,tl); end
end
end