# 97 — GitHub 협업 가이드 (5명 동시 작업)

> Final_Project.slx은 거의 수정하지 않고,
> 알고리즘은 `functions/*.m` 외부 파일로 분리하여 충돌을 회피한다.

---

## 1. 핵심 아이디어

### 문제
- Simulink `.slx`는 **binary** → git이 diff/merge 못 함
- 5명이 동시에 같은 모델 수정 → 누가 commit 하든 나머지 4명은 충돌

### 해결
- `.slx`는 **컨테이너**만 유지 (입출력 wiring + MATLAB Function block wrapper)
- 실제 알고리즘은 **외부 `.m`** 파일로 분리
- 각 dev는 자기 `.m`만 수정 → git이 text diff/merge 자동 처리

## 2. 폴더 구조

```
Practice_sample/
├── .gitattributes        ← .slx binary 명시
├── .gitignore            ← slprj/, *.slxc, _backup 제외
└── src_cm4sl/
    ├── generic_IVS.mdl   ← 거의 안 건드림 (Supervisor만)
    ├── setup_paths.m     ← 새 세션 시 1회 실행
    ├── functions/        ← ★ 5명이 각자 자기 .m만 수정 ★
    │   ├── README.md     (인터페이스 명세)
    │   ├── mode_manager_fcn.m                (Dev-B)
    │   ├── path_planning_fcn.m               (Dev-D)
    │   ├── trajectory_planning_fcn.m         (Dev-C)
    │   ├── longitudinal_controller_fcn.m     (Dev-A)
    │   └── lateral_controller_fcn.m          (Dev-E)
    └── Models/
        ├── Day3/, Day4_5/, Day6/             ← 학습용 (수정 가능)
```

## 3. Final Project 모델 구조

```
generic_IVS/CarMaker/Subsystem/Final Project/Scenario 1
│
├── [Do Not Modify 영역]   (좌측, x<2500)
│   ├── Read CM Dict ×5    (Car.Fr1.tx/ty/rz, Car.v, DevDist)
│   ├── Read CM Dict (Traffic.T22~T28.rzv)
│   ├── Bus Creator ×3
│   └── Goto 태그 발행
│
└── [Modify 영역]   (우측, x>2500)
    ├── Lib_ModeManager            ← Dev-B
    │     └── Wrapper_mode_manager_fcn  (MATLAB Function block)
    ├── Lib_PathPlanning           ← Dev-D
    │     └── Wrapper_path_planning_fcn
    ├── Lib_TrajectoryPlanning     ← Dev-C
    │     └── Wrapper_trajectory_planning_fcn
    ├── Lib_LongitudinalController ← Dev-A
    │     └── Wrapper_longitudinal_controller_fcn
    └── Lib_LateralController      ← Dev-E
          └── Wrapper_lateral_controller_fcn
```

### Wrapper 블록의 코드 (예: Lib_LateralController 안)
```matlab
function steer_cmd = fcn(cte, mode)
%#codegen
% IMPORTANT: To modify algorithm, edit functions/lateral_controller_fcn.m
% in the repo, NOT this block. This minimizes .slx changes.
steer_cmd = lateral_controller_fcn(cte, mode);
end
```

→ Wrapper는 한 줄짜리 forwarder. 알고리즘은 외부 .m에서.

## 4. 누가 무엇을 수정하는가

| Role | 수정하는 파일 | 수정하지 말 것 |
|---|---|---|
| **Dev-A** | `functions/longitudinal_controller_fcn.m` | 그 외 모두 |
| **Dev-B** | `functions/mode_manager_fcn.m` | 그 외 모두 |
| **Dev-C** | `functions/trajectory_planning_fcn.m` | 그 외 모두 |
| **Dev-D** | `functions/path_planning_fcn.m` | 그 외 모두 |
| **Dev-E** | `functions/lateral_controller_fcn.m` | 그 외 모두 |
| **Dev-F** | `generic_IVS.mdl` (wiring 시), `functions/README.md`, 통합/검증 | functions/*.m 직접 수정 X |

## 5. Git 워크플로

### 초기 셋업 (각 dev 1회)
```bash
git clone <repo>
cd Practice_sample
# MATLAB 시작 → src_cm4sl 로 cd 후:
#   run setup_paths.m
#   open generic_IVS.mdl
```

### 일상 작업
```
git pull origin main
# functions/<자기 파일>.m 수정 + CarMaker 시뮬 테스트
git add functions/<자기 파일>.m
git commit -m "feat(dev-X): improve algorithm"
git push
```

### .slx 수정이 필요할 때 (rare)
- 입출력 wiring 추가 (예: 새 신호 필요) → **반드시 Dev-F와 상의**
- PR 브랜치로 작업 + 다른 dev의 작업 멈춤 요청
- 머지 후 모두 git pull

## 6. 시그니처 변경 (충돌 위험 ↑)

외부 함수의 **입출력 개수/이름 변경**은 wrapper와 .slx 둘 다 영향.

### 절차
1. PR로 제안 + Dev-F 리뷰
2. 합의되면: Dev-F가 .slx wrapper 수정 + PR merge
3. 모두 git pull 후 자기 .m 업데이트

### 절대 금지
- 합의 없이 시그니처 변경 → 다른 dev 시뮬 깨짐
- .slx wrapper 직접 수정 (Dev-F만)

## 7. 검증 체크리스트

```matlab
%% 새 외부 함수 작성 후 단위 테스트
steer = lateral_controller_fcn(0.5, int32(1));
assert(abs(steer) <= pi/6, '조향각 한계 초과');
```

`git status` 깨끗한지 (자기 .m 외 변경 0개)

## 8. 트러블슈팅

| 증상 | 원인 | 해결 |
|---|---|---|
| `Undefined function 'xxx_fcn'` | functions/ path 누락 | `addpath('functions')` 또는 `setup_paths.m` 실행 |
| 시뮬 시 `lock` 에러 | generic_IVS 안 닫고 모델 수정 | `close_system('generic_IVS',0)` 후 수정 |
| 머지 충돌 (.slx) | 두 dev가 같이 .slx 수정 | Dev-F가 한쪽 수동 적용 + 다른 쪽 retry |
| 시그니처 mismatch | wrapper와 .m 입출력 불일치 | functions/README.md 확인 후 일치시킴 |

---

## 9. 실제 신호 매핑 (셋업 완료 후)

### Goto/From 신호 dictionary

| Goto 태그 | Producer | Consumer | 타입 | 비고 |
|---|---|---|---|---|
| `Ego_Velocity` | Read CM Dict5 (`Car.v`) | Lib_Long, Lib_Mode, Lib_Traj | double | 자차 속도 [m/s] |
| `Ego_Global_Pos` | Bus Creator3 (`Fr1.tx/ty/rz`) | Lib_Mode, Lib_Path, Lib_Traj | Bus | `{x, y, yaw}` |
| `Simulation_Time` | Read CM Dict4 (`Time`) | - | double | 사용 가능 |
| `Obstacle_Info` | Subsystem | Lib_Mode, Lib_Path | Bus | 구조 미확인 (TODO) |
| `waypoints` | FromWorkspace | - | array | 사용 가능 |
| `do_Logic_10ms_1` | FunctionCallSplit2 | Lib_Long, Lib_Lat (Trigger) | function-call | 10ms 주기 |
| `do_Logic_10ms_2` | FunctionCallSplit2 | Lib_Mode (Trigger) | function-call | 10ms |
| `do_Logic_100ms_1` | FunctionCallSplit4 | Lib_Path (Trigger) | function-call | 100ms |
| `do_Logic_100ms_2` | FunctionCallSplit4 | Lib_Traj (Trigger) | function-call | 100ms |
| **`Mission_Mode`** | Lib_ModeManager | Lib_Long, Lib_Lat, Lib_Traj | int32 | 신규 — Mode 출력 |
| **`Trajectory_CTE`** | Lib_TrajectoryPlanning | Lib_LateralController | double | 신규 — CTE 보정 |
| **`Target_Lane`** | Lib_PathPlanning | Lib_ModeManager | int32 | 신규 — 목표 차선 |
| **`Lane_Cost`** | Lib_PathPlanning | (참고용) | double [4×1] | 신규 — 차선 비용 |
| **`Accel_Cmd`** | Lib_LongitudinalController | (Write CM Dict 최종 연결) | double | m/s², 신규 |
| **`Steer_Cmd`** | Lib_LateralController | (Write CM Dict 최종 연결) | double | rad, 신규 |

### Lib별 내부 wiring (셋업 완료 상태)

```
Lib_LongitudinalController (10ms)
  From[Ego_Velocity]      ──┐
  Constant 20             ──┤  Wrapper_longitudinal_controller_fcn  ──→ Goto[Accel_Cmd]
  Constant 999            ──┤   ↳ functions/longitudinal_controller_fcn.m
  From[Mission_Mode]      ──┘

Lib_LateralController (10ms)
  From[Trajectory_CTE]    ──┐  Wrapper_lateral_controller_fcn      ──→ Goto[Steer_Cmd]
  From[Mission_Mode]      ──┘   ↳ functions/lateral_controller_fcn.m

Lib_TrajectoryPlanning (100ms)
  Constant 0              ──┐
  From[Mission_Mode]      ──┤
  From[Ego_Global_Pos]──BusSel─→[x,y,yaw]──┤  Wrapper_trajectory_planning_fcn ──→ Goto[Trajectory_CTE]
  From[Ego_Velocity]      ──┘   ↳ functions/trajectory_planning_fcn.m

Lib_PathPlanning (100ms)
  From[Ego_Global_Pos]──BusSel─→[x,y,yaw]──┐
  Constant 0 (traffic_x)  ──┤  Wrapper_path_planning_fcn  ──→ Goto[Target_Lane]
  Constant 0 (traffic_y)  ──┤   ↳ functions/path_planning_fcn.m  ──→ Goto[Lane_Cost]
  Constant int32(1)       ──┘

Lib_ModeManager (10ms)
  From[Ego_Velocity]      ──┐
  From[Ego_Global_Pos]──BusSel─→[x,y,yaw]──┤
  Constant 999 (traffic_x)──┤  Wrapper_mode_manager_fcn  ──→ Goto[Mission_Mode]
  Constant 0 (traffic_y)  ──┤   ↳ functions/mode_manager_fcn.m
  From[Target_Lane]       ──┤
  Constant int32(1)       ──┘
```

## 10. ⚠️ TODO — 각 Dev가 해야 할 일

### 🔴 공통 TODO (Dev-F 우선)
- [ ] `Obstacle_Info` Bus 구조 확인 + 각 Lib 안에 BusSelector 추가 (현재 traffic_x/y는 Constant placeholder)
- [ ] `Car.Road.Path.DevDist` Read CM Dict 추가 → Goto[CrossTrackError] → Lib_TrajectoryPlanning 입력 (현재는 0)
- [ ] `current_lane` 계산 모듈 추가 (현재 Lib_Path/Mode 모두 Constant int32(1))
- [ ] `Accel_Cmd`, `Steer_Cmd` → Write CM Dict 연결 (현재 발행만 됨, Sink 없음)
  - `Car.CFL.rz_ext`, `Car.CFR.rz_ext` ← Steer_Cmd
  - `AccelCtrl.DesiredAx` ← Accel_Cmd

### 🟡 각 Dev TODO (자기 .m만 수정)
- **Dev-A**: `longitudinal_controller_fcn.m`에 곡률 감속 + ACC 강화
- **Dev-B**: `mode_manager_fcn.m`에 톨게이트/주차 전환 로직 추가
- **Dev-C**: `trajectory_planning_fcn.m`에 Werling Frenet 다중 후보 (DEV_C v2 patch 참고)
- **Dev-D**: `path_planning_fcn.m`에 lane_cost 정밀 계산 (LaneRiskBus 영감)
- **Dev-E**: `lateral_controller_fcn.m`에 path yaw feedforward 추가 (필요시)

---

## 11. ✅ 셋업 완료 (CarMaker 시뮬 가능)

### 추가된 Write CM Dict

| 블록 이름 | CarMaker 변수 | 출처 | Saturation |
|---|---|---|---|
| `Write_AccelCmd` | `AccelCtrl.DesiredAx` | `From[Accel_Cmd]` → `Sat_Accel` | [-5.0, +3.0] m/s² |
| `Write_SteerL` | `Car.CFL.rz_ext` | `From[Steer_Cmd]` → `Sat_Steer` | [-π/6, +π/6] rad |
| `Write_SteerR` | `Car.CFR.rz_ext` | `From[Steer_Cmd]` → `Sat_Steer` | [-π/6, +π/6] rad |

### 전체 신호 흐름

```
┌─ CarMaker ─────────────────────────────────┐
│ Car.Fr1.tx/ty/rz, Car.v, Time             │
└──┬─────────────────────────────────────────┘
   │ Read CM Dict ×5 + BusCreator3
   ↓
Goto[Ego_Velocity], Goto[Ego_Global_Pos], ...
   │
   ├─→ Lib_PathPlanning (100ms) ──→ Goto[Target_Lane], Goto[Lane_Cost]
   ├─→ Lib_ModeManager (10ms) ────→ Goto[Mission_Mode]
   ├─→ Lib_TrajectoryPlanning (100ms) → Goto[Trajectory_CTE]
   ├─→ Lib_LateralController (10ms) → Goto[Steer_Cmd]
   └─→ Lib_LongitudinalController (10ms) → Goto[Accel_Cmd]
                                                  │
                                                  ↓
                       Saturation (안전 한계) → Write CM Dict ×3
                                                  │
                                                  ↓
┌─ CarMaker ─────────────────────────────────┐
│ AccelCtrl.DesiredAx, Car.CFL/CFR.rz_ext   │
└────────────────────────────────────────────┘
```

### 시뮬 실행 (각 dev 누구나)

```matlab
%% MATLAB Command Window
cd D:\\HL_IVS_School\\...\\Practice_sample\\src_cm4sl
run setup_paths.m       %% cmenv + functions/ path 추가
open generic_IVS.mdl    %% 메인 모델
%% CarMaker GUI에서: Select TestRun → day7_final → Start
```

### 현재 동작 상태 (placeholder 알고리즘으로)
- 목표 속도: 20 m/s (Lib_Long Const_VTarget)
- 횡 제어: PID (Kp=1.5) — 단, `Trajectory_CTE` 입력이 0이므로 직진만
- 모드: Lane Keep만 (트래픽 placeholder)
- → **차량이 직진 가속** 후 등속 주행

각 dev가 자기 .m 파일을 정교화하면 실제 자율주행 동작이 나옴.

---

## 12. ⚠️ Day 시나리오 활성/비활성 관리

### 문제
`generic_IVS.mdl` 안에는 Day3, Day6, Final Project 시나리오가 **모두 동거**합니다.
동시에 활성 상태면 시뮬 시작할 때 **모든 시나리오의 Read CM Dict 변수**가
존재해야 합니다 — 하지만 TestRun (예: `day7_final`)에는 그 TestRun에 정의된
변수만 있어서, 다른 시나리오의 변수 (예: `Traffic.T01.rzv`)는 없음 → 에러.

### 해결: 사용하지 않는 시나리오는 commented out

| 시나리오 | 사용 TestRun | 평소 상태 |
|---|---|---|
| `Day3/Scenario 1` | `day3_scenario1` 외 | 🔒 commented |
| `Day3/Scenario 2` | `day3_scenario2` 외 | 🔒 commented |
| `Day6/Scenario 1` | `day6_scenario1` 외 | 🔒 commented |
| `Final Project/Scenario 1` | `day7_final` | ✅ **활성** (default) |

### 학습용 Day 시뮬을 원할 때

임시로 해당 Day를 활성화 + Final Project를 비활성화:
```matlab
%% Day6 시뮬할 때만
set_param('generic_IVS', 'Lock', 'off');
set_param('generic_IVS/CarMaker/Subsystem/Day6/Scenario 1', 'Commented', 'off');
set_param('generic_IVS/CarMaker/Subsystem/Final Project/Scenario 1', 'Commented', 'on');
save_system('generic_IVS');
%% Day6 시뮬 끝나면 원상복귀!
set_param('generic_IVS/CarMaker/Subsystem/Day6/Scenario 1', 'Commented', 'on');
set_param('generic_IVS/CarMaker/Subsystem/Final Project/Scenario 1', 'Commented', 'off');
save_system('generic_IVS');
```

### Git 관점
- **default branch**: Final Project 활성, Day 모두 비활성
- 다른 dev pull 시 이 상태가 보장됨
- Day 시뮬 후엔 반드시 원상복귀 후 commit

---

## 13. ⚠️ 셋업 과정에서 만난 4가지 함정 (해결됨)

### 함정 #1: From Workspace의 waypoints 변수 형식
- **증상**: `시간 값은 감소하지 않아야 합니다` 에러
- **원인**: base workspace의 `waypoints`가 [x, y] 877×2 형식인데
  From Workspace 블록은 첫 column을 시간으로 해석
- **해결**: `setup_paths.m`이 자동으로 [time, x, y] 877×3 형식으로 변환

### 함정 #2: Bus 신호가 Goto/From 통과 시 type 손실
- **증상**: `BusSelector 입력이 Bus가 아니다` 에러
- **원인**: BusCreator3 → Goto[Ego_Global_Pos] → From → BusSelector 경로에서
  Bus type 정보가 정확히 전달 안 됨
- **해결**: BusSelector 제거 + 우측 영역에 새 Read CM Dict 3개 추가
  - `Read_Ego_X_Lib` → `Goto[Ego_Global_X]`
  - `Read_Ego_Y_Lib` → `Goto[Ego_Global_Y]`
  - `Read_Ego_Yaw_Lib` → `Goto[Ego_Yaw]`

### 함정 #3: Goto/From이 atomic subsystem 경계 못 넘음
- **증상**: `Goto/From connections cannot cross nonvirtual subsystem boundaries`
- **원인**: Lib_*는 Function-Call Triggered Subsystem = 강제 nonvirtual atomic
- **해결**: Lib 내부의 From/Constant 모두 제거, Inport/Outport로 명시 전달
  - Lib 안: Inport → Wrapper → Outport
  - Lib 밖: From/Constant → Lib.Inport, Lib.Outport → Goto

### 함정 #4: Function-call signal 단순 분기 불가
- **증상**: `function-call signal is branched incorrectly`
- **원인**: 같은 트리거 신호(`do_Logic_10ms_1`)를 2개 Lib(Long, Lat)가 받을 때
  단순 line branch로는 불가, Function-Call Split 블록 필요
- **해결**: `FCS_10ms_LongLat` Function-Call Split 블록 추가 (1 in → 2 out)

### Goto 태그 dictionary (최종 정리)

| Goto 태그 | 생성 위치 | 소비 위치 | 비고 |
|---|---|---|---|
| `Ego_Global_X` | `Read_Ego_X_Lib` (신규) | Lib_Mode/Path/Traj 입력 | Bus 우회용 |
| `Ego_Global_Y` | `Read_Ego_Y_Lib` (신규) | 위 동일 | Bus 우회용 |
| `Ego_Yaw` | `Read_Ego_Yaw_Lib` (신규) | 위 동일 | Bus 우회용 |
| `Ego_Velocity` | Read CM Dict5 (기존) | Lib_Long/Mode/Traj | 그대로 |
| `Mission_Mode` | Lib_ModeManager 출력 | Lib_Long/Lat/Traj | 신규 |
| `Trajectory_CTE` | Lib_TrajectoryPlanning 출력 | Lib_LateralController | 신규 |
| `Target_Lane` | Lib_PathPlanning 출력 | Lib_ModeManager | 신규 |
| `Lane_Cost` | Lib_PathPlanning 출력 | (참고용) | 신규 vector[4] |
| `Accel_Cmd` | Lib_LongitudinalController 출력 → Goto | Saturation → Write CM Dict | 신규 |
| `Steer_Cmd` | Lib_LateralController 출력 → Goto | Saturation → Write CM Dict | 신규 |
