%% MotionPlanningControl 프로젝트 시작 스크립트
%% 사용법: 이 파일을 실행 (F5) 또는 MATLAB Command Window 에서
%%        run('startup_project.m')

projRoot = 'C:\Users\hajin\Downloads\MotionPlanningControl-main\MotionPlanningControl-main\02_Carmaker_project\Practice_sample';

%% 1) 작업폴더를 src_cm4sl 로 이동
cd(fullfile(projRoot, 'src_cm4sl'));
fprintf('✅ Working dir: %s\n', pwd);

%% 2) CarMaker 환경 초기화 (CarMaker MATLAB / CM4SL 경로 추가 + cminit)
cmenv;

%% 3) Subsystem Reference 모델들 (각 Day) 경로 추가
modelsRoot = fullfile(projRoot, 'src_cm4sl', 'Models');
dayFolders = dir(modelsRoot);
for k = 1:length(dayFolders)
    if dayFolders(k).isdir && ~startsWith(dayFolders(k).name, '.')
        addpath(fullfile(modelsRoot, dayFolders(k).name));
    end
end
fprintf('✅ Models 폴더 경로 추가 완료\n');

%% 4) 메인 모델 열기
open_system('generic_IVS');
fprintf('✅ generic_IVS 모델 열림\n');

%% 5) 일관성 검증
try
    set_param('generic_IVS', 'SimulationCommand', 'update');
    fprintf('✅ Update Diagram 통과 — 시뮬레이션 준비 완료\n');
catch ME
    fprintf('⚠️ Update Diagram 오류: %s\n', ME.message);
end

fprintf('\n=== 🚀 다음 단계 ===\n');
fprintf('  1) Apps → CarMaker → Build\n');
fprintf('  2) CarMaker GUI → TestRun 선택 → SIM_START\n');
