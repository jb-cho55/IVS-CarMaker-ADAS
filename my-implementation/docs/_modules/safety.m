function [CFL, CFR, Ax, Gear] = safety(Steer, Accel, Ego, OF, Path, Mode, P_Ax, P_Sfl, P_Sfr, P_Gear)
%#codegen
persistent prev_st; if isempty(prev_st); prev_st=0; end
ev=Ego(4); tx=Ego(1); ty=Ego(2); ua=Mode(4);
Np=numel(Path)/2; No=floor(numel(OF)/8); slen=zeros(Np,1); px=tx;py=ty;
for j=1:Np; qx=Path(2*j-1);qy=Path(2*j); seg=sqrt((qx-px)^2+(qy-py)^2); if j==1; slen(j)=seg; else; slen(j)=slen(j-1)+seg; end; px=qx;py=qy; end
emerg=false;
for i=1:No
  o=(i-1)*8; if OF(o+8)<0.5; continue; end
  ox=OF(o+1); oy=OF(o+2); xr=OF(o+3); yr=OF(o+4); vrel=OF(o+5); vyr=OF(o+6); dist=OF(o+7);
  if ua>0.5
    if dist<0.8; emerg=true; end
  else
    if dist<1.4 && ev>5; emerg=true; end
    if ev>5 && dist<5.5 && abs(xr)<7 && abs(yr)<4 && yr*vyr<-0.4; emerg=true; end
    dmin=inf; jm=1; for j=1:Np; dx=Path(2*j-1)-ox; dy=Path(2*j)-oy; d=dx*dx+dy*dy; if d<dmin; dmin=d; jm=j; end; end
    dcross=sqrt(dmin); s=slen(jm);
    if dcross<1.8 && s>0.3 && s<25; ttc=inf; if vrel<-0.2; ttc=s/(-vrel); end; if ttc<1.0 || s<3.5; emerg=true; end; end
  end
end
ax=Accel; if emerg; ax=-4.0; end; if ax>3; ax=3; elseif ax<-4; ax=-4; end
st=prev_st+0.12*(Steer-prev_st); if st>0.5; st=0.5; elseif st<-0.5; st=-0.5; end; prev_st=st;
if Mode(5)>0.5
  CFL=P_Sfl; CFR=P_Sfr; Ax=P_Ax; Gear=P_Gear;
else
  CFL=st; CFR=st; Ax=ax; Gear=1.0;
end
end