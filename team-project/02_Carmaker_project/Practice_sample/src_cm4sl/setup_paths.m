%% setup_paths.m — 새 세션 시작 시 1회 실행
%%   1) CarMaker 환경 (cmenv)
%%   2) functions/ 폴더 path 추가
%%   3) waypoints 변수 [time, x, y] 형식으로 자동 변환

thisDir = fileparts(mfilename('fullpath'));
cd(thisDir);

%% --- 1) CarMaker 환경 ---
if exist('cmenv', 'file') == 2
    cmenv;
end

%% --- 2) functions/ path ---
fcnDir = fullfile(thisDir, 'functions');
if exist(fcnDir, 'dir') && ~contains(path, fcnDir)
    addpath(fcnDir);
    fprintf('functions/ added to MATLAB path\n');
end

%% --- 3) waypoints 변환 (From Workspace 블록용) ---
wpMatFile = fullfile(thisDir, 'map_data', 'final', 'Final_ver_waypoints.mat');
if exist(wpMatFile, 'file')
    wp_data = load(wpMatFile);
    if isfield(wp_data, 'waypoints')
        xy = wp_data.waypoints;
        N = builtin('size', xy, 1);
        Ts_wp = 0.1;
        t_col = (0:N-1)' * Ts_wp;
        waypoints = [t_col, xy];
        assignin('base', 'waypoints', waypoints);
        fprintf('waypoints converted: Nx3 [time, x, y]\n');
        if isfield(wp_data, 'ids')
            assignin('base', 'wp_ids', wp_data.ids);
        end
    end
else
    fprintf('WARNING: Final_ver_waypoints.mat not found\n');
end

fprintf('Setup complete. Now: open generic_IVS.mdl\n');
