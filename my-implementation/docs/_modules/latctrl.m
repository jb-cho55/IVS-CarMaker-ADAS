function steer = latctrl(Traj, Ego, WB)
%#codegen
tx=Ego(1); ty=Ego(2); rz=Ego(3); lax=Traj(1); lay=Traj(2);
c=cos(rz); s=sin(rz); ex=lax-tx; ey=lay-ty;
xv=c*ex+s*ey; yv=-s*ex+c*ey; Ld=sqrt(xv*xv+yv*yv); if Ld<1e-3; Ld=1e-3; end
alpha=atan2(yv,xv); delta=atan2(2*WB*sin(alpha),Ld);
if delta>0.5; delta=0.5; elseif delta<-0.5; delta=-0.5; end
steer=delta;
end