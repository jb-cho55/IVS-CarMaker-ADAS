function Traj = trajplan(Path, Ego, Mode, OF)
%#codegen
Np=numel(Path)/2; v=Ego(4); tx=Ego(1); ty=Ego(2); erz=Ego(3);
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
  No=floor(numel(OF)/10); lead_s=inf; lead_vrel=0;
  for i=1:No
    o=(i-1)*10; if OF(o+8)<0.5; continue; end
    ox=OF(o+1); oy=OF(o+2); dmin=inf; jm=1;
    for j=1:Np; dx=Path(2*j-1)-ox; dy=Path(2*j)-oy; d=dx*dx+dy*dy; if d<dmin; dmin=d; jm=j; end; end
    if sqrt(dmin)<1.6 && slen(jm)>1 && slen(jm)<55 && slen(jm)<lead_s; lead_s=slen(jm); lead_vrel=OF(o+5); end
  end
  if isfinite(lead_s); des=6+1.2*v; va=v+lead_vrel+0.45*(lead_s-des); if va<0; va=0; end; if va<ts; ts=va; end; end

  % --- predictive footprint avoidance: project ego (along path @ current speed) and
  %     each obstacle (@ its global velocity) forward; if a predicted oriented-box
  %     contact is found, slow JUST enough (proportional, creep-floored) to avoid it by
  %     timing. Covers crossing / overtake / abeam; gentle -> no hard-brake deadlock. ---
  egoL=4.68; egoW=1.88; obsL=4.47; obsW=1.97; Th=1.6; ns=6;
  epx=zeros(ns,1); epy=zeros(ns,1); eth=zeros(ns,1);
  for st=1:ns
    starg=v*(Th*st/ns); ej=1;
    for jj=1:Np; if slen(jj)<=starg; ej=jj; end; end
    if ej<1; ej=1; elseif ej>Np; ej=Np; end
    e0=ej-1; if e0<1; e0=1; end; e1=ej+1; if e1>Np; e1=Np; end
    eth(st)=atan2(Path(2*e1)-Path(2*e0), Path(2*e1-1)-Path(2*e0-1));
    epx(st)=Path(2*ej-1)+egoL/2*cos(eth(st)); epy(st)=Path(2*ej)+egoL/2*sin(eth(st));
  end
  gpred=inf;
  for i=1:No
    o=(i-1)*10; if OF(o+8)<0.5; continue; end
    ox=OF(o+1); oy=OF(o+2); dist=OF(o+7); vgx=OF(o+9); vgy=OF(o+10);
    if dist>20.0; continue; end
    osp=sqrt(vgx*vgx+vgy*vgy); oth=erz; if osp>0.5; oth=atan2(vgy,vgx); end
    for st=1:ns
      tau=Th*st/ns; opx=ox+vgx*tau; opy=oy+vgy*tau;
      dx=opx-epx(st); dy=opy-epy(st); ct=cos(eth(st)); stt=sin(eth(st));
      rx=ct*dx+stt*dy; ry=-stt*dx+ct*dy; thR=oth-eth(st);
      gp=tp_obb(rx,ry,thR, egoL/2,egoW/2, obsL/2,obsW/2);
      if gp<gpred; gpred=gp; end
    end
  end
  gsafe=1.2;
  if gpred<gsafe
    fr=gpred/gsafe; if fr<0; fr=0; end
    tsv=ts*(0.25+0.75*fr); if tsv<0.8; tsv=0.8; end
    if tsv<ts; ts=tsv; end
  end
else
  % --- APPROACH lateral dodge: ego must NOT stop at the branch (stopping ->
  %     ego-coupled traffic stalls -> deadlock). Instead steer laterally past a
  %     path-blocking dynamic object while keeping speed. Localized to the
  %     low-curvature entry straight (a strong offset on the bend would derail).
  %     Perception-based: dodge to the side OPPOSITE the object's signed
  %     path-lateral position; magnitude fades with along-path distance. ---
  No=floor(numel(OF)/10);
  kf=0;
  for j=2:min(Np-1,14)
    a1x=Path(2*j-1)-Path(2*j-3); a1y=Path(2*j)-Path(2*j-2); a2x=Path(2*j+1)-Path(2*j-1); a2y=Path(2*j+2)-Path(2*j);
    l1=sqrt(a1x*a1x+a1y*a1y); l2=sqrt(a2x*a2x+a2y*a2y);
    if l1>0.1 && l2>0.1; dth=atan2(a2y,a2x)-atan2(a1y,a1x); if dth>pi; dth=dth-2*pi; elseif dth<-pi; dth=dth+2*pi; end; kk=abs(dth)/((l1+l2)*0.5); if kk>kf; kf=kk; end; end
  end
  bestw=0; bestlat=0;
  if kf<0.03
    for i=1:No
      o=(i-1)*10; if OF(o+8)<0.5; continue; end
      ox=OF(o+1); oy=OF(o+2); xr=OF(o+3); dist=OF(o+7);
      if dist>16 || xr<-1.5; continue; end
      dmin=inf; jm=1; for j=1:Np; dx=Path(2*j-1)-ox; dy=Path(2*j)-oy; d=dx*dx+dy*dy; if d<dmin; dmin=d; jm=j; end; end
      dcross=sqrt(dmin); s=slen(jm);
      if dcross<3.5 && s>0.2 && s<16
        j0=max(1,jm-1); j1=min(Np,jm+1);
        thp=atan2(Path(2*j1)-Path(2*j0), Path(2*j1-1)-Path(2*j0-1));
        dx=ox-Path(2*jm-1); dy=oy-Path(2*jm);
        lato=-sin(thp)*dx+cos(thp)*dy;
        w=(16-s)/16;
        if w>bestw; bestw=w; bestlat=lato; end
      end
    end
  end
  if bestw>0
    dr=-1; if bestlat<0; dr=1; end
    doff=dr*2.2*bestw;
    j0=1; for jj=1:Np; if slen(jj)<=Ld; j0=jj; end; end
    j0b=max(1,j0); j1=min(Np,j0b+1);
    thp=atan2(Path(2*j1)-Path(2*j0b), Path(2*j1-1)-Path(2*j0b-1));
    lax=lax-sin(thp)*doff; lay=lay+cos(thp)*doff;
  end
end
Traj=[lax; lay; ts; sf];
end

function gp = tp_obb(cx,cy,th, eh,ew, oh,ow)
%#codegen
EC=[eh ew; eh -ew; -eh -ew; -eh ew];
ct=cos(th); st=sin(th); OC=zeros(4,2); OL=[oh ow; oh -ow; -oh -ow; -oh ow];
for i=1:4; OC(i,1)=cx+ct*OL(i,1)-st*OL(i,2); OC(i,2)=cy+st*OL(i,1)+ct*OL(i,2); end
ax4=[1 0; 0 1; ct st; -st ct]; ov=true; pen=inf;
for a=1:4
  axx=ax4(a,1); axy=ax4(a,2);
  e1=EC(:,1)*axx+EC(:,2)*axy; o1=OC(:,1)*axx+OC(:,2)*axy;
  lo=max(min(e1),min(o1)); hi=min(max(e1),max(o1)); ovl=hi-lo;
  if ovl<=0; ov=false; break; end
  if ovl<pen; pen=ovl; end
end
if ov; gp=-pen; else
  gp=inf;
  for ii=1:4
    for jj=1:4
      kk=mod(jj,4)+1;
      gp=min(gp, tp_ps(EC(ii,1),EC(ii,2),OC(jj,1),OC(jj,2),OC(kk,1),OC(kk,2)));
      gp=min(gp, tp_ps(OC(ii,1),OC(ii,2),EC(jj,1),EC(jj,2),EC(kk,1),EC(kk,2)));
    end
  end
end
end

function d = tp_ps(px,py, ax,ay, bx,by)
%#codegen
abx=bx-ax; aby=by-ay; den=abx*abx+aby*aby; tt=0.0;
if den>0; tt=((px-ax)*abx+(py-ay)*aby)/den; if tt<0; tt=0; elseif tt>1; tt=1; end; end
qx=ax+tt*abx; qy=ay+tt*aby; d=hypot(px-qx,py-qy);
end
