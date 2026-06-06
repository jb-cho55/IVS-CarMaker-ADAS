function R = ivs_eval(N, tcap, tag)
%#ok<*AGROW>
% IVS_EVAL  Run the IVS mission N times and aggregate reliability metrics.
%   R = ivs_eval(N, tcap, tag).  tcap = sim-time cap [s] per run. tag = label.
%   Robust to the scenario's run-to-run RNG: reports per-run + distribution.
%   PRIMARY metric = CarMaker collision sensor (Sensor.Collision.Vhcl.Fr1.Count,
%   the IPGMovie red-flag ground truth): a run PASSES only if Count stays 0.
%   Also reports footprint overlap (rect_gap) as a secondary diagnostic.
if nargin<1, N=5; end
if nargin<2, tcap=200; end
if nargin<3, tag='run'; end
mdl='generic_IVS';
root='C:\Users\gmkk6\Desktop\last_dance\02_Carmaker_project\Practice_sample\SimOutput';
gx=21.4; gy=-44.3; entx=6.0; enty=-36.6; tollx=-176.17; tolly=-85.66; brx=-49.253; bry=17.335;
egoL=4.68; egoW=1.88; obsL=4.47; obsW=1.97;
R=struct('tend',{},'nToll',{},'nBranch',{},'reachEnt',{},'tEnt',{},'nColl',{},'collObj',{},...
         't22gap',{},'mingap',{},'minobj',{},'nOverlap',{},'stopPct',{},'vToll',{});
fprintf('\n===== ivs_eval [%s] N=%d tcap=%d =====\n', tag, N, tcap);
for r=1:N
  cmguicmd('SaveMode save',5000); cmguicmd('LoadTestRun IVS_Final_Project',25000); pause(1);
  set_param(mdl,'SimulationCommand','start'); pause(2);
  for k=1:120
    st=get_param(mdl,'SimulationStatus'); tt=get_param(mdl,'SimulationTime');
    if ~strcmp(st,'running'), break; end
    if tt>=tcap, break; end
    pause(2);
  end
  set_param(mdl,'SimulationCommand','stop'); pause(3);
  d=dir(fullfile(root,'**','*.erg')); [~,ix]=sort([d.datenum],'descend');
  D=cmread(fullfile(d(ix(1)).folder,d(ix(1)).name));
  t=double(D.Time.data(:)); x=double(D.Car_Fr1_tx.data(:)); y=double(D.Car_Fr1_ty.data(:));
  yaw=double(D.Car_Fr1_rz.data(:)); v=double(D.Car_v.data(:)); dt=median(diff(t));
  % --- PRIMARY: CarMaker collision sensor (ground truth, = IPGMovie red flag) ---
  nColl=0; collt=NaN;
  if isfield(D,'Sensor_Collision_Vhcl_Fr1_Count')
    cc=double(D.Sensor_Collision_Vhcl_Fr1_Count.data(:)); nColl=max(cc);
    jh=find(cc>0,1); if ~isempty(jh); collt=t(jh); end
  end
  nToll=numel(find(diff([0;(abs(x-tollx)<6 & abs(y-tolly)<40)])==1));
  nBr  =numel(find(diff([0;(hypot(x-brx,y-bry)<6 & t>15)])==1));
  dent=hypot(x-entx,y-enty); [de,je]=min(dent); reachEnt=double(de<3.0); tEnt=t(je)*reachEnt;
  % which traffic object is nearest at the moment of first collision (for diagnosis)
  collObj=0;
  if nColl>0 && ~isnan(collt)
    jc=find(t>=collt,1); dmn=inf;
    for k=0:28
      fx=sprintf('Traffic_T%02d_tx',k); if ~isfield(D,fx), continue; end
      tx=double(D.(fx).data(:)); ty=double(D.(sprintf('Traffic_T%02d_ty',k)).data(:));
      dd=hypot(x(jc)-tx(jc), y(jc)-ty(jc)); if dd<dmn; dmn=dd; collObj=k; end
    end
  end
  % footprint gaps (secondary): ALL frames in each object's encounter window, TRUE heading.
  t22gap=inf; mingap=inf; minobj=0; nOv=0;
  for k=16:28
    fx=sprintf('Traffic_T%02d_tx',k); if ~isfield(D,fx), continue; end
    tx=double(D.(fx).data(:)); ty=double(D.(sprintf('Traffic_T%02d_ty',k)).data(:));
    rzf=sprintf('Traffic_T%02d_rz',k); hasrz=isfield(D,rzf); trz=zeros(numel(t),1); if hasrz, trz=double(D.(rzf).data(:)); end
    dd=hypot(x-tx,y-ty); win=find(dd<7); if isempty(win), continue; end
    gbest=inf;
    for w=1:numel(win)
      j=win(w);
      if hasrz, thT=trz(j); else, i1=max(1,j-3); i2=min(numel(t),j+3); thT=atan2(ty(i2)-ty(i1),tx(i2)-tx(i1)); end
      cE=[x(j)+egoL/2*cos(yaw(j)), y(j)+egoL/2*sin(yaw(j))];
      gp=rect_gap(cE,yaw(j),egoL/2,egoW/2,[tx(j) ty(j)],thT,obsL/2,obsW/2);
      if gp<gbest, gbest=gp; end
    end
    if gbest<0.5, nOv=nOv+1; end
    if gbest<mingap, mingap=gbest; minobj=k; end
    if k==22, t22gap=gbest; end
  end
  nT2=abs(x-tollx)<15 & abs(y-tolly)<50; vT=NaN; if any(nT2), vT=min(v(nT2)); end
  R(r)=struct('tend',t(end),'nToll',nToll,'nBranch',nBr,'reachEnt',reachEnt,'tEnt',tEnt,...
     'nColl',nColl,'collObj',collObj,'t22gap',t22gap,'mingap',mingap,'minobj',minobj,...
     'nOverlap',nOv,'stopPct',100*mean(v<0.5),'vToll',vT);
  fprintf('  run%d: toll=%d br=%d ent=%d(t%.0f) | COLLISION=%d(T%02d,t%.0f) | nOvlp=%d minGap=%+.2f(T%02d) stop=%.0f%%\n',...
    r,nToll,nBr,reachEnt,tEnt,nColl,collObj,collt,nOv,mingap,minobj,R(r).stopPct);
end
% aggregate
tolls=[R.nToll]; ents=[R.reachEnt]; ovl=[R.nOverlap]; t22=[R.t22gap]; mg=[R.mingap]; sp=[R.stopPct]; coll=[R.nColl];
fprintf('--- AGG [%s] ---\n', tag);
fprintf('  *** CarMaker COLLISION-FREE: %d/%d (%.0f%%)  [runs with Count>0: %d] ***\n', sum(coll==0), N, 100*mean(coll==0), sum(coll>0));
fprintf('  toll==1 (lap-1, no relap): %d/%d (%.0f%%)\n', sum(tolls==1), N, 100*mean(tolls==1));
fprintf('  reached entrance        : %d/%d (%.0f%%)\n', sum(ents==1), N, 100*mean(ents==1));
fprintf('  runs with footprint ovl : %d/%d (%.0f%%)\n', sum(ovl>0), N, 100*mean(ovl>0));
fprintf('  min footprint gap: mean=%.2f  worst=%.2f\n', mean(mg(isfinite(mg))), min(mg));
fprintf('  stopped%%: mean=%.0f  range=[%.0f,%.0f]\n', mean(sp), min(sp), max(sp));
end
