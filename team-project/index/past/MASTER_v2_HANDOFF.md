# 🎯 MASTER v2 — Supervisor Handoff

> **From**: Dev-F (Supervisor)
> **To**: 전체 팀 (Dev-A, B, C, D, E)
> **Date**: 2026-05-26
> **Status**: 인프라 셋업 완료, 알고리즘 구현 단계 시작

---

## 1. 한 줄 요약

`generic_IVS.mdl`의 `Final Project/Scenario 1` 안 5개 `Lib_*` 컨테이너에 wrapper가 셋업됐고,
각자 `functions/<자기파일>.m`만 수정하면 됩니다. `.slx`는 건드리지 마세요.

---

## 2. 팀원 배정 (1대1 매핑)

| Dev | 담당 `.m` 파일 | Simulink Lib 위치 | 주기 |
|:-:|---|---|:-:|
| **A** | `functions/longitudinal_controller_fcn.m` | `Lib_LongitudinalController` | 10ms |
| **B** | `functions/mode_manager_fcn.m` | `Lib_ModeManager` | 10ms |
| **C** | `functions/trajectory_planning_fcn.m` | `Lib_TrajectoryPlanning` | 100ms |
| **D** | `functions/path_planning_fcn.m` | `Lib_PathPlanning` | 100ms |
| **E** | `functions/lateral_controller_fcn.m` | `Lib_LateralController` | 10ms |
| **F** | 통합/검증/`.slx` 관리 | — | — |

---

## 3. 첫 셋업 (각 dev 1회만)

```bash
# 1) 저장소 clone
git clone <repo-url>
cd Practice_sample

# 2) MATLAB 시작 → Command Window에서:
cd src_cm4sl
run setup_paths.m       % cmenv + functions/ path + waypoints 변환 자동
open generic_IVS.mdl
```

`setup_paths.m`이 자동으로 처리:
- CarMaker 환경 (`cmenv`)
- `functions/` 폴더를 MATLAB path에 추가
- `waypoints` 변수 [time, x, y] 형식으로 변환 (Final_ver_waypoints.mat 로드)

---

## 4. 신호 흐름 (전체)

```
┌─ CarMaker (vehicle dynamics + scenario) ─────────────────┐
│ Car.Fr1.tx/ty/rz, Car.v, Time                            │
└──┬──────────────────────────────────────────────────────────┘
   │ Read CM Dict (기존 5개 + 신규 3개)
   ↓
Goto[Ego_Velocity, Ego_Global_X, Ego_Global_Y, Ego_Yaw, ...]
   │
   ↓ (외부 From → Lib.Inport)
┌─────────────────────────────────────────────────────────────┐
│ Lib_PathPlanning (100ms)        →  Goto[Target_Lane, Lane_Cost]   │
│    ↳ functions/path_planning_fcn.m  (Dev-D)                  │
│                                                             │
│ Lib_ModeManager (10ms)          →  Goto[Mission_Mode]       │
│    ↳ functions/mode_manager_fcn.m  (Dev-B)                  │
│                                                             │
│ Lib_TrajectoryPlanning (100ms)  →  Goto[Trajectory_CTE]     │
│    ↳ functions/trajectory_planning_fcn.m  (Dev-C)           │
│                                                             │
│ Lib_LateralController (10ms)    →  Goto[Steer_Cmd]          │
│    ↳ functions/lateral_controller_fcn.m  (Dev-E)            │
│                                                             │
│ Lib_LongitudinalController (10ms) → Goto[Accel_Cmd]         │
│    ↳ functions/longitudinal_controller_fcn.m  (Dev-A)       │
└─────────────────────────────────────────────────────────────┘
   │
   ↓ Saturation (안전 한계)
Write CM Dict (Car.CFL/CFR.rz_ext, AccelCtrl.DesiredAx)
   ↓
┌─ CarMaker ───────────────────────────────────────────────────┐
│ 차량 거동                                                    │
└─────────────────────────────────────────────────────────────┘
```

---

## 5. 모든 Goto 태그 (signal dictionary)

| 태그 | 생성 | 소비 | 타입 | 단위 |
|---|---|---|---|---|
| `Ego_Velocity` | Read CM Dict5 | Lib_Long/Mode/Traj | double | m/s |
| `Ego_Global_X` | Read_Ego_X_Lib | Lib_Mode/Path/Traj | double | m |
| `Ego_Global_Y` | Read_Ego_Y_Lib | Lib_Mode/Path/Traj | double | m |
| `Ego_Yaw` | Read_Ego_Yaw_Lib | Lib_Mode/Path/Traj | double | rad |
| `Simulation_Time` | Read CM Dict4 | (사용 가능) | double | s |
| `Mission_Mode` | Lib_ModeManager | Lib_Long/Lat/Traj | int32 | enum (1~9) |
| `Trajectory_CTE` | Lib_TrajectoryPlanning | Lib_Lat | double | m |
| `Target_Lane` | Lib_PathPlanning | Lib_Mode | int32 | 1~4 |
| `Lane_Cost` | Lib_PathPlanning | (참고용) | double[4] | unitless |
| `Accel_Cmd` | Lib_Long | Write CM Dict | double | m/s² |
| `Steer_Cmd` | Lib_Lat | Write CM Dict | double | rad |

### Mission_Mode enum
```matlab
int32(1) = LANE_KEEP  (default)
int32(2) = OVERTAKE   (decision in progress)
int32(3) = LC_OUT     (lane change 시작)
int32(4) = LC_PASS    (옆 차선에서 추월 중)
int32(5) = LC_BACK    (원래 차선 복귀)
int32(6) = TOLLGATE
int32(7) = PARKING
int32(9) = EMERGENCY
```

---

## 6. 일상 작업 워크플로

### Step 1: pull
```bash
git pull origin main
```

### Step 2: MATLAB 환경 셋업 (매 세션 1회)
```matlab
cd D:\\HL_IVS_School\\...\\src_cm4sl
run setup_paths.m
open generic_IVS.mdl
```

### Step 3: 자기 `.m` 수정 + 시뮬 테스트
```matlab
%% (예: Dev-A) functions/longitudinal_controller_fcn.m 편집
%% CarMaker GUI: day7_final TestRun → Start
```

### Step 4: commit + push
```bash
git status      # only your .m should be modified
git add functions/<your-file>.m
git commit -m "feat(dev-X): describe change"
git push
```

---

## 7. 규칙 (Golden Rules)

1. **자기 `.m`만 수정** — `git status`에 다른 파일 modified 있으면 STOP
2. **`.slx` 수정 금지** — 입출력 wiring 변경 필요하면 Dev-F에게 PR
3. **시그니처 변경 금지** — 입력/출력 개수/타입 변경은 PR + 합의
4. **`%#codegen` directive 유지** — MATLAB Function block에서 호출 가능해야 함
5. **고정 크기 배열만** — variable-size 금지 (codegen 안 됨)
6. **persistent 변수**는 OK, 초기화는 `if isempty(...)` 패턴
7. **Day 시나리오 활성화 토글 시 원복** — 학습 후 Final Project로 다시

---

## 8. 검증 체크리스트 (commit 전)

```matlab
%% 1) 단위 테스트 (각자)
% Dev-A 예:
acc = longitudinal_controller_fcn(10, 20, 50, int32(1));
assert(acc >= -5 && acc <= 3, 'acc 범위 위반');

%% 2) 모델 Update Diagram
set_param('generic_IVS', 'SimulationCommand', 'update');
%% 에러 없어야 함

%% 3) CarMaker 시뮬 (day7_final) 끝까지 abort 없이 통과
```

```bash
# 4) git status — 자기 .m만 modified
git status
```

---

## 9. 현재 상태 (placeholder 알고리즘)

### 동작 상태
- 목표 속도 20 m/s 로 가속
- 횡 제어 단순 PID (Trajectory_CTE = 0 입력이라 직진)
- Mission Mode는 LK ↔ LC_OUT ↔ LC_PASS ↔ LC_BACK 전환 (트래픽 placeholder 999m이라 발동 안 됨)
- Path Planning은 항상 target_lane = 1 반환

### 즉시 알고리즘 작업이 가능한 항목
- 모든 Lib에 placeholder 코드가 있음 → 각자 정교화
- 외부 .m만 수정하면 즉시 시뮬에 반영됨 (rebuild 불필요)

---

## 10. 단계별 통합 계획 (제안)

| Phase | 기간 | 작업 | Owner |
|:-:|:-:|---|---|
| **P0** | 완료 | 인프라/wrapper/wiring 셋업 | Dev-F |
| **P1** | Week 1 | Dev-A: 종방향 PID + ACC 정교화 | A |
| **P2** | Week 1 | Dev-E: 횡방향 Pure Pursuit/Stanley 추가 | E |
| **P3** | Week 2 | Dev-B: Mode Manager + 추월 결정 | B |
| **P4** | Week 2-3 | Dev-D: Lane Cost + 차선 선택 | D |
| **P5** | Week 3-4 | Dev-C: Werling Frenet 다중 후보 | C |
| **P6** | Week 4 | 톨게이트 + 주차 시퀀스 추가 | A~F |
| **P7** | Week 5 | E2E 튜닝 + 시뮬 검증 | All |

---

## 11. 알려진 함정 (이미 해결, 참고)

| 함정 | 해결 |
|---|---|
| `waypoints` 변수 시간 감소 에러 | `setup_paths.m` 자동 변환 |
| Bus 신호 Goto/From type 손실 | Read CM Dict 3개 추가 |
| Goto/From atomic 경계 못 넘음 | Inport/Outport로 명시 전달 |
| function-call 단순 분기 오류 | Function-Call Split 추가 |
| Day3/Day6 시나리오 활성화로 변수 못 찾음 | Final Project만 활성 |

---

## 12. 관련 문서

| 문서 | 용도 |
|---|---|
| **MASTER_v2_HANDOFF.md** | 이 문서 (전체 브리핑) |
| `97_Github_Collab_Guide.md` | Git/협업 상세 (트러블슈팅 포함) |
| `DEV_TASKS_v2.md` | 5명 dev 각자의 task brief |
| `functions/README.md` | 함수 인터페이스 명세 |
| `04_System_Interface_v2_PATCH.md` | LaneRiskBus/lane_cost (참고) |
| `DEV_B_Overtaking_v2_PATCH.md` | lane_cost argmin 알고리즘 (참고) |
| `DEV_C_LaneChange_Tollgate_v2_PATCH.md` | Werling Frenet 알고리즘 (참고) |
| `99_Integration_Plan_v2_PATCH.md` | 정규화/DVA 검증 (참고) |

## 13. 질문/이슈

- `.slx` 수정 필요 → Dev-F에게 직접 ping
- 시그니처 변경 → GitHub PR 생성 + 전체 리뷰
- 모델 에러 디버깅 → 97 가이드 §13 참조 후 Dev-F
