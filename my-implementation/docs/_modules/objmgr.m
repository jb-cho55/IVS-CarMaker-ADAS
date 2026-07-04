function ObsFeat = objmgr(Obs, Ego)
%#codegen
ex=Ego(1); ey=Ego(2); erz=Ego(3); ev=Ego(4); c=cos(erz); s=sin(erz);
ObsFeat=zeros(232,1);
for i=1:29
  if i<=16; b=(i-1)*5; gx=Obs(b+1); gy=Obs(b+2); vgx=0; vgy=0;
  else; b=80+(i-17)*10; gx=Obs(b+1); gy=Obs(b+2); vgx=Obs(b+4); vgy=Obs(b+5); end
  dx=gx-ex; dy=gy-ey; xr=c*dx+s*dy; yr=-s*dx+c*dy;
  vxr=c*vgx+s*vgy-ev; vyr=-s*vgx+c*vgy; dist=sqrt(dx*dx+dy*dy);
  vv=0; if (abs(gx)>0.1||abs(gy)>0.1) && dist<200; vv=1; end
  o=(i-1)*8; ObsFeat(o+1)=gx; ObsFeat(o+2)=gy; ObsFeat(o+3)=xr; ObsFeat(o+4)=yr; ObsFeat(o+5)=vxr; ObsFeat(o+6)=vyr; ObsFeat(o+7)=dist; ObsFeat(o+8)=vv;
end
end