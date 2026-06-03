function cl_plot(tnum, subdir)
%CL_PLOT  Plot actual parking path + obstacles + goal for a saved T##.mat.
if nargin<2, subdir='round1'; end
D=fullfile('C:\Users\User\Desktop\IVS\_clone_0603\02_Carmaker_project\Practice_sample\FP_test_results',subdir);
S=load(fullfile(D,sprintf('T%02d.mat',tnum))); R=S.R;
X=S.X; Y=S.Y; YAW=S.YAW; ST=S.ST;
g=R.goal; gx=g(1); gy=g(2); gd=g(3);

% obstacle list (pp_obstacles), footprint obs_L=4.47 ahead, obs_W=1.97
H=pi/2;
OB=[7.3 -28.7 -H;12.8 -6.8 -H;21.3 -6.6 -H;30.0 -6.5 -H;41.7 -6.3 -H;6.9 -6.9 -H;
    7.0 -21.8 H;18.6 -21.8 H;24.3 -21.9 H;36.0 -21.9 H;38.8 -21.8 H;41.8 -21.8 H;
    12.6 -28.8 -H;24.3 -28.9 -H;41.6 -28.9 -H;9.9 -44.4 H;21.4 -44.3 H;24.5 -44.4 H;
    36.0 -44.4 H;41.8 -44.5 H;44.6 -44.4 H];
obsL=4.47; obsW=1.97;

f=figure('Visible','off','Position',[100 100 900 750]); hold on; axis equal;
% lot bound
rectangle('Position',[2 -45.5 46 42.5],'EdgeColor',[.6 .6 .6],'LineStyle','--');
% obstacles (rear-bumper ref: spans [0,obsL] fwd, +-obsW/2)
for k=1:size(OB,1)
  c=rectc(OB(k,1),OB(k,2),OB(k,3),obsL,0,obsW/2);
  patch(c(:,1),c(:,2),[.75 .8 .9],'EdgeColor',[.4 .5 .7]);
  text(OB(k,1),OB(k,2),sprintf('%d',k),'FontSize',7,'Color',[.3 .3 .5],'HorizontalAlignment','center');
end
% actual path: driving (ST<3) light, parking (ST>=3) bold red
dr=ST<3; pk=ST>=3;
plot(X(dr),Y(dr),'-','Color',[.6 .8 .6],'LineWidth',1);
plot(X(pk),Y(pk),'-','Color',[.85 .1 .1],'LineWidth',1.8);
% start, final
plot(X(1),Y(1),'go','MarkerFaceColor','g','MarkerSize',7);
plot(X(end),Y(end),'ks','MarkerFaceColor','k','MarkerSize',7);
% goal slot (ego footprint at goal: rear-bumper ref, veh_L=4.68 ahead, veh_W=1.88)
cg=rectc(gx,gy,deg2rad(gd),4.68,0,1.88/2);
patch(cg(:,1),cg(:,2),'y','FaceAlpha',0.25,'EdgeColor',[.9 .6 0],'LineWidth',2);
quiver(gx,gy,2.5*cos(deg2rad(gd)),2.5*sin(deg2rad(gd)),0,'Color',[.9 .5 0],'LineWidth',2,'MaxHeadSize',2);
% final ego footprint (actual parked)
cf=rectc(X(end),Y(end),YAW(end),4.68,0,1.88/2);   % YAW is in RADIANS (Car_Fr1_rz)
plot(cf([1:4 1],1),cf([1:4 1],2),'k-','LineWidth',1.5);

title(sprintf('T%02d  goal(%.1f,%.1f,%+d°)  parked(%.2f,%.2f,%+.1f°)  perr=%.2fm yerr=%+.1f° %s',...
  tnum,gx,gy,gd,R.px,R.py,R.pyaw,R.perr,R.yerr,R.verdict),'FontSize',10);
xlabel('X [m]'); ylabel('Y [m]'); grid on;
legend({'lot','','','','','','','','','','','','','','','','','','','','','','drive','park','start','final'},'Location','eastoutside');
xlim([0 49]); ylim([-46 -2]);
out=fullfile(D,sprintf('T%02d.png',tnum));
print(f,out,'-dpng','-r90'); close(f);
fprintf('saved %s\n',out);
end

function c=rectc(x,y,yaw,ahead,behind,halfw)
co=cos(yaw); si=sin(yaw);
lx=[ahead ahead -behind -behind]; ly=[halfw -halfw -halfw halfw];
c=zeros(4,2);
for k=1:4, c(k,1)=x+co*lx(k)-si*ly(k); c(k,2)=y+si*lx(k)+co*ly(k); end
end
