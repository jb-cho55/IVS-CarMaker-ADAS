# Day4_5 주차 플래너 작업 진행상황 (WORK_PROGRESS)

> 단일 진행로그. 주요 변경/진단/결정마다 갱신.
> 최종 갱신: 2026-05-30 (세션 진행 중)

---

## 🎯 ROOT CAUSE 2 (2026-05-30) — 플래너가 넓은 맵에서 무한루프/행
- 실측(오프라인 직접 호출, 현재 복원 Parking.m):
  - 맵 **[2.5,49.5]×[-2,-48]**(라이브) → `rrt_ccrs_plan` **첫 호출에서 안 끝남(행/무한루프)**. 프로세스가 파일 잡고 안 죽음 → kill 필요.
  - 맵 **[4,48]×[-46.8,-4]**(검증) → `path_len=153`, hang 없음, desired_ax 유효(isnan=0).
- 결과: 라이브 맵에서 MF5 첫 스텝이 영영 안 끝남 → **desired_ax가 아예 안 나옴**(사용자: 디스플레이 "값이 안나와"). = 증상 일치.
- **픽스: .slx 맵 경계 상수 8개를 [4,48]로 복원**(set_map_good.m). start(5.5,-36.5)+goal(17,-7)+map[4,48] = run_rrt_real/test_integrated_move로 검증된 설정.
- ⚠️ 주의: 넓은 맵에서 플래너가 무한루프 = 코드 버그. **핸드오프 시작점(1.819,-36.641)은 x<4라 넓은 맵이 필요** → 그 통합 전에 플래너 무한루프 원인(샘플링/start-corridor/RS 엔드게임)을 잡아야 함. 지금은 [4,48]로 먼저 차를 움직이게 함.
- desired_ax가 첫 ~0.5s 0인 건 정상(기어 시프트 50틱 게이트). 이후 R 넣고 후진 시작.

## ⚠️ 교정 (2026-05-30, 실측 기반) — DVA는 틀렸음, User 모드가 정답
- 실측: **DVA 모드는 수동 AccelCtrl.DesiredAx=1.0 주입에도 무반응**. **User 모드는 차가 움직임**. → 이 CM4SL 셋업에선 **`DesrAccelFunc=User`가 정상 동작 모드** (User 함수가 Simulink/외부 DesiredAx를 구동에 반영). DVA 변경은 잘못 → User로 원복(사용자 수행, 유지).
- 따라서 **구동 체인은 정상**(User 모드 + 수동 주입으로 차 움직임 확인). 남은 문제는 **MF5가 desired_ax=0을 내보냄**.
- 직접 원인: Parking.m L138 `stay_put = path_len<=2 && 두 점 거의 동일` → `v_des=0` → `desired_ax=0`. 즉 **플래너가 퇴화 경로(2점 스텁) 반환 = 경로 탐색 실패**. (코드 주석도 "path_len<2면 경로 못 찾음"이라 명시)
- **다음 확인**: 라이브 Scope(path_len_dbg) 값이 ≤2(플래너 실패) vs 큰 값(컨트롤 이슈). 오프라인선 라이브 맵[2.5,49.5]에서 스텁 발생 → 맵 경계 변경이 플래너 실패 원인 의심(이전 성공은 [4,48]).

## ✅ 현재 픽스 + 오프라인 검증 통과 (2026-05-30)
- 맵 [4,48] 복원 후 현재 Parking.m로 **오프라인 end-to-end: path_len=153, 차량 3.40m 이동, t=1.29s 후진 시작, selector D→R, max 0.30m/s**. hang 없음.
- 남은 건 라이브 검증: generic_IVS **리로드(bdclose all→재오픈)** + **User 모드 유지** + SIM_START. 첫 ~14s 플래닝(프리즈) → ~0.5s 기어 R 전환 → 후진 시작.

## 0. 현재 상태 한 줄 요약
- **근본원인 확정 + 수정 완료**: Kia_EV6 `AccelCtrl.DesrAccelFunc`를 **`User`→`DVA`** 로 변경(백업 `Kia_EV6.bak_before_DVA_20260530`). 이제 CarMaker가 우리 `AccelCtrl.DesiredAx`를 읽음.
- 안전성 검증: **모든 Day 시나리오(Day2/Day3_2/Day4_5/Day6_1/Day6_2)가 동일하게 AccelCtrl.DesiredAx를 사용**, 커스텀 User AccelCtrl C함수는 없음 → DVA가 전 시나리오에 정합적, 다른 day를 깨지 않음.
- 목표 (17,−7) 복원·적용 완료. Parking.m 전진성공 버전 복원·재배포 완료. 플래너 정상.
- **다음**: generic_IVS 리로드 + SIM_START로 실차 검증(차가 움직이는지).

## ROOT CAUSE (2026-05-30 확정)
- `Data/Vehicle/Kia_EV6` L624-625:
  - `VehicleControl.0.Kind = AccelCtrl` (AccelCtrl가 종방향 제어 담당)
  - **`AccelCtrl.DesrAccelFunc = User`** ← DVA 아님! (git HEAD에서도 줄곧 User. 바뀐 건 `i` 1.0→0.1뿐)
- RM §7.4.3: DVA여야 외부(Simulink) `AccelCtrl.DesiredAx`를 읽음. User면 사용자 C 함수가 결정 → 우리 write 무시.
- 과거 전진성공 = VC.Gas/Brake(페달) 구동 시절. 인터페이스 단일화로 VC.Gas(Dict18)/VC.Brake(Dict16) Commented → 구동원 소멸.
- 배선/플래너/차트는 정상. 문제는 **AccelCtrl 모드**.
- **의존성 검증 결과**: Day2/Day3_2/Day4_5/Day6_1/Day6_2 전부 `AccelCtrl.DesiredAx` write 사용. `src_cm4sl/*.c`에 커스텀 User AccelCtrl relay 함수 없음 → User 모드는 우리 DesiredAx를 구동에 반영 못 함.
- **결정·적용(2026-05-30)**: `AccelCtrl.DesrAccelFunc = User → DVA` 변경(백업 보관). 전 시나리오가 DesiredAx를 쓰므로 DVA가 정합·안전. `AccelCtrl.i=0.1`, VC 등 나머지는 그대로(추가 수정 안 함).
- ⚠️ 다른 vehicle 파일을 쓰는 day가 있다면 그 파일도 동일 변경 필요(현 TestRun들은 모두 Kia_EV6).

---

## 1. 시스템 구조 (확정)
- **실제 실행 모델 = `generic_IVS`** (`src_cm4sl/generic_IVS.mdl`, OPC 텍스트 패키지).
- 그 안 `CarMaker/Subsystem/Day4 & 5/Scenario 1` = **Subsystem Reference** → `Day4_5_Scenario_1.slx`.
  - 즉 내가 수정/배포해온 `Day4_5_Scenario_1.slx`가 **올바른 대상**.
  - ⚠️ Subsystem Reference라서 generic_IVS가 **메모리에 옛 사본을 들고 있으면 디스크 수정이 반영 안 됨** → `bdclose all` 후 재오픈 필요.
- occupancy grid는 `generate_map_`에서 **고정 전역 프레임 x∈[0,100], y∈[-100,0]**, RES=0.5, 200×200.
  `map_boundary`는 그 안의 주행가능 폴리곤만 정의(폴리곤 밖 = occupied).

## 2. Day4_5_Scenario_1.slx 내부 MATLAB 함수 (전수 조사)
| chart | 블록 | 역할 | 상태 |
|---|---|---|---|
| 131 | MATLAB Function1 | `generate_map_` | 활성 |
| 140 | MATLAB Function2 | `add_obstacle_` | 활성 |
| 109 | **MATLAB Function5** | **Parking (RRT*+CCRS, LIVE)** | **활성** ← 배포대상 |
| 86 | MATLAB Function | `path_planner_` | 비활성(Commented) |
| 64 | MATLAB Function3 | Parking 중복 | 비활성 |
| 96 | MATLAB Function4 | 컨트롤러 중복 | 비활성 |

### 배선 (확정, 정상)
- MF5 출력 → `Write CM Dict19 = AccelCtrl.DesiredAx` ✓, `Dict17 = DM.SelectorCtrl` ✓, `Dict14/15 = Car.CFL/CFR.rz_ext`(조향) ✓
- VC.Gas(Dict18)/VC.Brake(Dict16) = Commented(의도된 비활성, 인터페이스 단일화).
- MF5 입력 8개 모두 연결: ego_x/y/yaw/vx, Start_Point, Finish_Point, goal_yaw, occ_map.
- **결론: 인터페이스/배선은 멀쩡함. "안 움직임"의 원인이 아님.**

## 3. 라이브 상수값 (Day4_5_Scenario_1.slx에서 직접 추출) — 핵심 이상치
| 상수 | 값 | 비고 |
|---|---|---|
| Start_Point | **(5.5, -36.5)** | 내 핸드오프 변경(1.819,-36.641) **미반영** (setup 스크립트만 수정, .slx엔 미적용) |
| Finish_Point | **(17, -11.7)** | setup의 (17,-7)과 **다름** — 누군가 -11.7로 변경 |
| goal_yaw | -pi/2 | |
| map_boundary | x∈[2.5,49.5], y∈[-48,-2] | setup 스크립트(0,48)와도 다름 |

> 참고: Parking.m에서 `start_point`은 미사용(planner는 live ego pose를 시작점으로 씀). 따라서 Start_Point 상수는 경로계획에 직접 영향 없음. 차량 스폰 위치는 TestRun의 `Vehicle.StartPos`가 결정.

## 4. 유력 원인 (검증 중)
- **(A) generic_IVS 메모리 stale** — 열린 모델이 옛 Day4_5 서브시스템 캐시 사용 → 차트 복원·상수 반영 안 됨. → 전진까지 멈춤 + 시작점 미반영을 동시 설명. **해결: `bdclose all` → 재오픈 → SIM_START.**
- **(B) 목표 (17,-11.7) 도달불가** → RRT* 빈 경로 → 컨트롤러 v_des≈0 → 안 움직임. **(오프라인 검증 진행 중: run_live_constants.m)**

## 5. 이번 세션 변경 이력
- Parking.m: 후진 권한 실험(V_TGT_MAX 0.45/KP 0.55/AX_MAX 0.30/AX_RISE 0.12, AX_START 0.08) → **전진성공 값으로 복원** (0.30/0.40/0.12/0.06, AX_START 0.03). REPLAN_PERIOD=2e9, EXTRA_AFTER_GOAL=600은 유지.
- 라이브 차트 재배포 완료 (48273 chars, F5_WRITE_DONE).
- setup_final_parking_test.m: 시작 (1.819,-36.641) + map x_min 0.0로 스크립트 수정(아직 .slx 미적용).

## 6. 다음 액션 (TODO)
- [ ] run_live_constants.m 결과로 (B) 확정/기각.
- [ ] (A) 확인: 사용자에게 `bdclose all`→재오픈→SIM_START 요청.
- [ ] 목표 (17,-11.7) 의도 확인 — 원래 (17,-7)로 되돌릴지.
- [ ] FP 통합(parking_scenario_fcn.m) / PR.
