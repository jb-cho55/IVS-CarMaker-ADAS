function ax = lonctrl(Traj, Ego)
%#codegen
target=Traj(3); v=Ego(4); ax=0.6*(target-v);
if ax>3; ax=3; elseif ax<-4; ax=-4; end
end