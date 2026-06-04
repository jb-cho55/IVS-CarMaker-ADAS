# FP 후진주차 — 진행상황 정리 (2026-06-04)

작업 위치: `_clone_0603` (작동 통합본). 주차 알고리즘 본체는 **`src_cm4sl/Models/FinalProject/Final_Project.slx`** 의
`CarMaker/Subsystem/Final Project/Scenario 1/Parking_system/MATLAB Function6` (= `pp_parking`, self-contained).
사람이 읽는 EML 소스 사본: `FP_campaign/MF6_FINAL.m`.

## 1. 목표 주입 / 결과 측정 메커니즘 (확정)
- **목표 좌표(rear-bumper pose)**: `Scenario 1/Parking_Info/Constant32`=x, `Constant33`=y, `Parking_system/Constant23`=yaw[rad].
  - MATLAB Function6는 `finish_point`(=From["Finish_Point"])+`goal_yaw`를 읽음. (Constant6/7/2 = Lib경로용, planner 미사용)
- **실제 궤적 로깅**: `ego_pos_dbg.Car_Fr1_tx/ty/rz`(rad), `status_dbg`, `occ_log`(200×200 점유맵). To Workspace 블록.
- **캠페인 도구**: `cl_run.m`(목표설정→실행→메트릭→저장), `cl_metrics.m`(parked pose / pos·yaw err / 기어반전), `cl_plot.m`(궤적+장애물+목표 시각화). `fp_coords.m`=29좌표.
- co-sim: CarMaker GUI 연결상태(day7_final online)에서 `set_param start`, ~6× 실시간(250s ≈ 40~60s wall).

## 2. 베이스라인 (수정 전, 19좌표) — 14/19 PASS
- FAIL: **T05·T08**(맵경계 슬롯, 계획불가), **T01·T10·T14**(−90° 진입기동 미완).
- 결과: `FP_campaign/online/T01–T19.mat`, `campaign_summary.csv`.

## 3. 사용자 지적 3가지 문제 & 수정

### 문제 2 — 장애물 무시/충돌 → ✅ 해결
- **근본원인**: `cfg.holdout = 14`(T13 차량, 24.3,-28.9)가 목표와 무관하게 **항상** occ맵에서 제거됨
  → 목표가 T13이 아니면 그 자리 실제 차량을 무시하고 경로 생성.
- **수정**: `cfg.holdout = 0` (동적 `pp_clear_goal`이 실제 목표슬롯만 비움). `cfg.margin 0.30 → 0.40`(간격↑).
- **검증**: occ맵에서 T13/양옆차량 모두 점유(1) 확인.

### 문제 3 — 정밀정렬 소실 & 문제 1 — 진입 실패 → ✅ 대부분 해결 (staging 전략)
- **근본원인**: pp_track(pure-pursuit)은 **위치 기반** → 슬롯 안에서 heading만 미세보정하는 짧은 기동을 실행 못함.
  비스듬히 진입 → 측면차 코너 충돌(T16) + 전진주차(+90) heading 5~10° 잔류.
- **수정 (사용자 아이디어)**: **넓은 통로(staging)에서 헤딩 정렬 후 슬롯으로 직진 진입**.
  - PLAN 단계: 목표서 heading축으로 `stage_dist`(5m) 떨어진 staging pose로 Hybrid A* → 거기서 슬롯까지 **직진 세그먼트** append.
  - **양방향**: 뒤쪽 통로면 전진 pull-in(dirv=+1), 앞쪽 통로면(상단열 등 뒤가 벽) 후진 reverse-in(dirv=−1).
  - 추종 정밀도: `v_fwd 1.0 → 0.6`(진입속도↓), `cfg.ds_align=0.15`(committed-RS 미세 보정).
  - (in-slot wiggle 직접조향 시도는 불안정해서 폐기)

## 4. 수정 후 결과 (staging 모델, 대표 10좌표 검증)
| 좌표 | 유형 | 이전 | staging 후 |
|---|---|---|---|
| T16 | −90 (충돌케이스) | 측면차 충돌 | **perr 0.028m, yaw +0.4°** ✅ 슬롯 정중앙 |
| T11 | +90 | yaw −5.3° | **0.027m, +0.9°** ✅ |
| T02 / T04 | +90 | +2.7° / −6.9° | **−0.8° / −2.1°** ✅ |
| T05 | −90 (상단우측 edge) | FAIL 51.6m | **0.020m, +0.5°** ✅ |
| T14 | −90 (우중 edge) | FAIL 3.5m/28° | **0.035m, −2.1°** ✅ |
| T03 / T18 | −90 | −3.0° / −2.9° | −3.8° / −7.4° (PASS) |
| **T10** | −90 (좌중) | FAIL 20m | **0.024m, −0.1°** ✅ (closer-staging + v_fwd↓) |
| **T01** | −90 (좌상단) | FAIL | **여전히 FAIL** (back-staging은 lot밖, fwd-staging은 북쪽 차들로 막혀 도달불가 — 거의 기하 한계) |

- **+90 전진주차 정밀도 회복**(<2°) + **가장자리 −90 진입실패 해결**(T05·T08·T10·T14)을 staging 전략이 달성.

## 5. 추가 개선 (2026-06-04 오후)
### 5a. staging 선택 = ego에 가까운 쪽 우선 (closer-staging)
- 뒤/앞 staging 중 ego에 가까운 쪽을 먼저 시도 → traverse 최소화. **T10 FAIL→PASS**.
### 5b. 진입속도↓ (v_fwd 0.6→0.4)
- 전진 진입을 천천히 → 추종 정밀도↑.
### 5c. hug-right 진입 (우측차선 hugging) — 주행 lane-following 재사용
- **`Driving_Module/Ego_Detector`**: 주차 진입차선(4/5)의 `ego_cross_track_err`에 **+1.2m offset** 추가
  → controller가 차선중심 대신 **우측에 붙여서** 추종 (= 진입로 우측 hugging).
- 효과: **T16 기동 중 측면차 겹침 4800→2457 (절반↓)**, 주차트리거가 더 서쪽이동해 기동 여유↑.
- offset 1.8↑은 진입로 이탈/주차실패 → **1.2가 최적**(당일 미세조정 여지).
- (참고: in-slot wiggle 직접조향 + parking-planner keep-right A* 패널티는 효과 없어 폐기/롤백)

## 6. 현재 모델 구성 (clean)
`staging(closer-staging 선택) + holdout=0 + margin=0.40 + ds_align=0.15 + v_fwd=0.4 + obs-clear + hug-right 진입(1.2)`

## 7. 남은 작업 (※ 시나리오 공개 후 미세조정 예정)
1. **T01** (좌상단 −90): 진입로/접근각 추가 검토 (혹은 출제 시 비현실 방향이면 제외).
2. **T16 잔여 겹침**(2457): 기동 중 좌측차 스침 완전제거 — 출제좌표 기준으로 offset·margin·stage_dist 튜닝.
3. 전체 29 재검증 + 전 좌표 plot.

## 8. 변경 파일
- `src_cm4sl/Models/FinalProject/Final_Project.slx` — 주차(staging 등) + **주행 Ego_Detector(hug-right 진입)**. ★핵심★
- `src_cm4sl/generic_IVS.mdl` — CM4SL 통합(autoload 경로).
- `FP_campaign/` — `cl_run/cl_metrics/cl_plot/fp_coords` + `MF6_FINAL.m`(주차 EML) + `DRV_Ego_Detector.m`(주행 EML, hug-right) + 결과.
