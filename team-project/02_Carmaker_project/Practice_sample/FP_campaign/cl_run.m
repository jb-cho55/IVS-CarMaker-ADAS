function R = cl_run(tnum, gx, gy, gdeg)
%CL_RUN  Set goal, run one CarMaker co-sim (robust launch+retry), save raw+metrics.
%  Clone FP goal: Parking_Info/Constant32=x, Constant33=y, Parking_system/Constant23=yaw[rad].
m   = 'generic_IVS';
S1  = [m '/CarMaker/Subsystem/Final Project/Scenario 1'];
PI  = [S1 '/Parking_Info'];
PS  = [S1 '/Parking_system'];
OUT = 'C:\Users\User\Desktop\IVS\_clone_0603\02_Carmaker_project\Practice_sample\FP_test_results';

% --- set goal (rear-bumper pose) ---
set_param([PI '/Constant32'],'Value',num2str(gx));
set_param([PI '/Constant33'],'Value',num2str(gy));
set_param([PS '/Constant23'],'Value',num2str(deg2rad(gdeg)));

% --- robust launch: verify sim actually advances; retry on simT=0 failures ---
launched = false; attempt = 0; tw = tic;
while ~launched && attempt < 4
    attempt = attempt + 1;
    if ~strcmp(get_param(m,'SimulationStatus'),'stopped')
        set_param(m,'SimulationCommand','stop'); pause(2);
    end
    pause(2);                                   % let CarMaker GUI settle between runs
    evalin('base','clear ego_pos_dbg status_dbg');
    set_param(m,'SimulationCommand','start');
    tl = tic;
    while toc(tl) < 12                          % launch window
        pause(1);
        stt  = get_param(m,'SimulationStatus');
        simt = get_param(m,'SimulationTime');
        if simt > 2.0, launched = true; break; end           % advancing -> launched
        if strcmp(stt,'stopped') && simt < 1.0 && toc(tl) > 3 % flipped back w/o moving
            break;
        end
    end
    if ~launched
        fprintf('  T%02d launch attempt %d FAILED (simT=%.2f); retry...\n', tnum, attempt, get_param(m,'SimulationTime'));
        if ~strcmp(get_param(m,'SimulationStatus'),'stopped'), set_param(m,'SimulationCommand','stop'); end
        pause(3);
    end
end

% --- poll to completion ---
while ~strcmp(get_param(m,'SimulationStatus'),'stopped') && toc(tw) < 130
    pause(2);
end
R.wall = toc(tw); R.run_status = get_param(m,'SimulationStatus');
R.attempts = attempt; R.launched = launched;
R.tnum = tnum; R.goal = [gx gy gdeg];

% --- read logged trajectory ---
epd = evalin('base','ego_pos_dbg'); sd = evalin('base','status_dbg');
x = epd.Car_Fr1_tx.Data; y = epd.Car_Fr1_ty.Data; yaw = epd.Car_Fr1_rz.Data;
T = epd.Car_Fr1_tx.Time;  st = sd.Data; stT = sd.Time;

if numel(x) < 100
    R.verdict = 'NO_RUN'; R.px=NaN;R.py=NaN;R.pyaw=NaN;R.perr=NaN;R.yerr=NaN;R.rev=NaN;
    R.has_park=false; R.entry=[NaN NaN NaN]; R.dmin=NaN;
    save(fullfile(OUT, sprintf('T%02d.mat',tnum)), 'R');
    fprintf('T%02d goal(%.2f,%.2f,%+d) | NO_RUN (log=%d samples, launched=%d, attempts=%d)\n', ...
        tnum, gx, gy, gdeg, numel(x), launched, attempt);
    return;
end

% --- metrics ---
M = cl_metrics(x,y,yaw,T,st,stT,gx,gy,gdeg);
fn = fieldnames(M); for i=1:numel(fn), R.(fn{i}) = M.(fn{i}); end

% --- save downsampled raw (50 Hz) ---
ds = 20;
X = x(1:ds:end); Y = y(1:ds:end); YAW = yaw(1:ds:end); TT = T(1:ds:end);
ST = interp1(stT, double(st), TT, 'previous', 'extrap');
if ~exist(OUT,'dir'), mkdir(OUT); end
save(fullfile(OUT, sprintf('T%02d.mat',tnum)), 'X','Y','YAW','TT','ST','R');

fprintf('T%02d goal(%.2f,%.2f,%+d) | parked(%.2f,%.2f,%+.1f) perr=%.3fm yerr=%+.1fdeg rev=%d | %-7s | %.0fs a%d\n', ...
    tnum, gx, gy, gdeg, R.px, R.py, R.pyaw, R.perr, R.yerr, R.rev, R.verdict, R.wall, attempt);
end
