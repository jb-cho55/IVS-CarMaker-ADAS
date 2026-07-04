function Mode = modemgr(Ego, OF, LANE_XY, LANE_LEN, cruise_lane, v_cruise, v_park, lane_width, branch_lane, toll_lane, toll_x, toll_y, branch_x, branch_y, entrance_x, entrance_y)
%#codegen
persistent committed toll_reached toll_done go_park on_appr lc_cool ph bypass astk
if isempty(committed); committed=double(cruise_lane); toll_reached=false; toll_done=false; go_park=false; on_appr=false; lc_cool=0; ph=Ego(3); bypass=false; astk=0; end
tx=Ego(1); ty=Ego(2); ev=Ego(4); nL=size(LANE_XY,3); No=floor(numel(OF)/8);
if lc_cool>0; lc_cool=lc_cool-1; end
dh=Ego(3)-ph; if dh>pi; dh=dh-2*pi; elseif dh<-pi; dh=dh+2*pi; end; ph=Ego(3); yawing=abs(dh)>0.004;
fwd=inf(nL,1); blocked=false(nL,1);
for i=1:No
  o=(i-1)*8; if OF(o+8)<0.5; continue; end
  ox=OF(o+1); oy=OF(o+2); xr=OF(o+3);
  bl=1; bd=inf;
  for li=1:nL
    n=LANE_LEN(li); dm=inf;
    for k=1:n; dx=LANE_XY(k,1,li)-ox; dy=LANE_XY(k,2,li)-oy; d=dx*dx+dy*dy; if d<dm; dm=d; end; end
    if dm<bd; bd=dm; bl=li; end
  end
  if sqrt(bd)>lane_width*0.6; continue; end
  if xr>0.5 && xr<fwd(bl); fwd(bl)=xr; end
  if xr>-13 && xr<22; blocked(bl)=true; end
end
cr=double(cruise_lane); dtoll=sqrt((tx-toll_x)^2+(ty-toll_y)^2); dbr=sqrt((tx-branch_x)^2+(ty-branch_y)^2);
if ~toll_done; if dtoll<8; toll_reached=true; end; if toll_reached && dtoll>15; toll_done=true; go_park=true; end; end
toll_lock = ~toll_done && dtoll<25;
if go_park && ~bypass && dbr<35 && ev<0.4; astk=astk+1; else; astk=0; end
if astk>160; bypass=true; on_appr=false; astk=0; committed=cr; end
if bypass && dbr>70; bypass=false; end
ovtk_ok = ~toll_lock && ~on_appr && lc_cool==0 && ~yawing && (~go_park || dbr>60 || bypass);
if ovtk_ok
  cl=committed;
  if fwd(cl)<35
    bestc=cl; bestg=fwd(cl);
    for dlt=[-1 1]
      cand=cl+dlt;
      if cand>=1 && cand<=nL && ~blocked(cand) && fwd(cand)>bestg+8; bestc=cand; bestg=fwd(cand); end
    end
    if bestc~=committed; committed=bestc; lc_cool=50; end
  elseif committed~=cr && ~blocked(cr) && fwd(cr)>40
    committed=cr; lc_cool=50;
  end
end
tl=committed; vset=v_cruise; stop=0; use_appr=0; pr=0;
if toll_lock; tl=double(toll_lane); committed=double(toll_lane); end
if go_park && ~bypass
  if on_appr || dbr<20
    on_appr=true; use_appr=1; vset=v_park; tl=double(branch_lane); committed=double(branch_lane);
  elseif dbr<40
    tl=double(branch_lane); committed=double(branch_lane); vset=8;
  elseif dbr<60
    tl=2; if nL>=2; committed=2; end; vset=10;
  else
    vset=10;
  end
  dent=sqrt((tx-entrance_x)^2+(ty-entrance_y)^2);
  if on_appr && dent<4.0; stop=1; if ev<0.3; pr=1; end; end
end
Mode=[tl; vset; stop; use_appr; pr];
end