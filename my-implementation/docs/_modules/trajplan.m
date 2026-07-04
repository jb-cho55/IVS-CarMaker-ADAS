function Traj = trajplan(Path, Ego, Mode, OF)
%#codegen
Np=numel(Path)/2; v=Ego(4); tx=Ego(1); ty=Ego(2);
Ld=6+0.6*v; if Ld<11; Ld=11; elseif Ld>18; Ld=18; end
px=tx;py=ty;accL=0;lax=Path(1);lay=Path(2); slen=zeros(Np,1);
for j=1:Np; qx=Path(2*j-1);qy=Path(2*j); seg=sqrt((qx-px)^2+(qy-py)^2); if j==1; slen(j)=seg; else; slen(j)=slen(j-1)+seg; end; accL=accL+seg; if accL<=Ld; lax=qx; lay=qy; end; px=qx;py=qy; end
ts=Mode(2); sf=Mode(3); ua=Mode(4); if sf>0.5; ts=0; end
if ua<0.5
  kmax=0;
  for j=2:min(Np-1,26)
    a1x=Path(2*j-1)-Path(2*j-3); a1y=Path(2*j)-Path(2*j-2); a2x=Path(2*j+1)-Path(2*j-1); a2y=Path(2*j+2)-Path(2*j);
    l1=sqrt(a1x*a1x+a1y*a1y); l2=sqrt(a2x*a2x+a2y*a2y);
    if l1>0.1 && l2>0.1; dth=atan2(a2y,a2x)-atan2(a1y,a1x); if dth>pi; dth=dth-2*pi; elseif dth<-pi; dth=dth+2*pi; end; kk=abs(dth)/((l1+l2)*0.5); if kk>kmax; kmax=kk; end; end
  end
  if kmax>0.002; vc=sqrt(3.0/kmax); if vc<ts; ts=vc; end; end
  No=floor(numel(OF)/8); lead_s=inf; lead_vrel=0;
  for i=1:No
    o=(i-1)*8; if OF(o+8)<0.5; continue; end
    ox=OF(o+1); oy=OF(o+2); dmin=inf; jm=1;
    for j=1:Np; dx=Path(2*j-1)-ox; dy=Path(2*j)-oy; d=dx*dx+dy*dy; if d<dmin; dmin=d; jm=j; end; end
    if sqrt(dmin)<1.6 && slen(jm)>1 && slen(jm)<55 && slen(jm)<lead_s; lead_s=slen(jm); lead_vrel=OF(o+5); end
  end
  if isfinite(lead_s); des=6+1.2*v; va=v+lead_vrel+0.45*(lead_s-des); if va<0; va=0; end; if va<ts; ts=va; end; end
end
Traj=[lax; lay; ts; sf];
end