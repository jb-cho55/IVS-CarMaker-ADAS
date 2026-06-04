function fp_run_campaign(m, idx, stopT, maxwall)
%FP_RUN_CAMPAIGN  Run ACTUAL CarMaker co-sim for the FP parking coords.
%   GUI MUST be connected first (open project -> model -> Open CarMaker GUI ->
%   "Open new GUI"). For each coord: set goal -> LoadTestRun day7_final ->
%   start -> poll to stop -> read newest erg -> metrics -> plot ACTUAL+PLANNED
%   -> save T##_actual.png + append campaign_actual.csv.
%
%   m       : model name (default 'generic_IVS')
%   idx     : row indices into fp_coords() (default = feasible: all except T05,T08)
%   stopT   : StopTime [s] (default 200)
%   maxwall : per-run wall-clock timeout [s] (default 260)
%
%   PRE-REQS: project open + GUI connected + base has waypoint_const + 18 buses
%     (run once:  load('...\src_cm4sl\map_data\final\waypoint_const.mat'); fp_define_buses; )
%   NOTE: if orphan HIL accumulates between runs, clean with (in OS shell, not MCP):
%         Stop-Process -Name HIL -Force
%   ⚠️ VALIDATE LIVE: StopTime / poll cadence may need tuning once real co-sim timing is seen.
if nargin<1 || isempty(m), m = 'generic_IVS'; end
camp = fileparts(mfilename('fullpath')); addpath(camp);
C = fp_coords();
if nargin<2 || isempty(idx), idx = find(~ismember(C(:,1),[5 8]))'; end   % skip known-infeasible
if nargin<3 || isempty(stopT), stopT = 200; end
if nargin<4 || isempty(maxwall), maxwall = 260; end

root  = fileparts(fileparts(get_param(m,'FileName')));   % ...\Practice_sample
outdir = fullfile(camp,'results'); if ~exist(outdir,'dir'), mkdir(outdir); end
S = load(fullfile(camp,'occ_mode1.mat')); om = S.om;
set_param(m,'StopTime',num2str(stopT));
csv = fullfile(outdir,'campaign_actual.csv');
fid = fopen(csv,'w'); fprintf(fid,'T,gx,gy,gdeg,err_m,yaw_deg,nrev,xf,yf,rzf,wall_s\n'); fclose(fid);

for k = idx
    Tn = C(k,1); gx = C(k,2); gy = C(k,3); gdeg = C(k,4);
    fprintf('\n=== T%02d goal(%.2f,%.2f,%.0f) ===\n', Tn, gx, gy, gdeg);
    fp_setgoal(m, gx, gy, gdeg);
    try, cmguicmd('LoadTestRun day7_final'); catch e, fprintf('LoadTestRun err: %s\n', e.message); end
    ed0 = dir(fullfile(root,'SimOutput','**','*.erg')); pre = 0; if ~isempty(ed0), pre = max([ed0.datenum]); end
    set_param(m,'SimulationCommand','start');
    tw = tic;
    while toc(tw) < maxwall
        st = get_param(m,'SimulationStatus'); simT = get_param(m,'SimulationTime');
        if strcmp(st,'stopped') && simT > 1, break; end
        if simT >= stopT - 0.5, break; end
        pause(2);
    end
    try, set_param(m,'SimulationCommand','stop'); catch, end
    pause(1); wall = toc(tw);
    ed = dir(fullfile(root,'SimOutput','**','*.erg'));
    [pn, ix] = max([ed.datenum]);
    if pn > pre
        ef = fullfile(ed(ix).folder, ed(ix).name);
        R = fp_metrics(ef, gx, gy, gdeg);
        % planned overlay: offline replan from the ACTUAL engage pose
        de = hypot(R.x-1.82, R.y+36.64); [~, ie] = min(de);
        [px,py,pth,~,np,ok] = plan_offline(R.x(ie), R.y(ie), R.rz(ie), gx, gy, gdeg*pi/180, om);
        if ok && np>=1, rbx = px(1:np)-0.95*cos(pth(1:np)); rby = py(1:np)-0.95*sin(pth(1:np)); else, rbx=[]; rby=[]; end
        mt.err = R.err; mt.yawerr = R.yawerr; mt.nrev = R.nrev;
        fp_plot(Tn, R.x, R.y, rbx, rby, gx, gy, gdeg, om, fullfile(outdir,sprintf('T%02d_actual.png',Tn)), mt);
        fid = fopen(csv,'a'); fprintf(fid,'%d,%.2f,%.2f,%.0f,%.3f,%.2f,%d,%.2f,%.2f,%.1f,%.0f\n', Tn,gx,gy,gdeg,R.err,R.yawerr,R.nrev,R.xf,R.yf,R.rzf,wall); fclose(fid);
        fprintf('T%02d DONE  err=%.3fm yaw=%.2fdeg rev=%d wall=%.0fs\n', Tn, R.err, R.yawerr, R.nrev, wall);
    else
        fprintf('T%02d NO fresh erg (run may have failed / GUI not connected)\n', Tn);
        fid = fopen(csv,'a'); fprintf(fid,'%d,%.2f,%.2f,%.0f,NaN,NaN,NaN,NaN,NaN,NaN,%.0f\n', Tn,gx,gy,gdeg,wall); fclose(fid);
    end
end
fprintf('\n=== campaign_actual done -> %s ===\n', csv);
end
