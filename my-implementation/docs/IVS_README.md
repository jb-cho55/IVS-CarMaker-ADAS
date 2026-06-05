# IVS Final Project — Autonomous Driving Stack (CM4SL)

자율주행 제어 스택을 `src_cm4sl/generic_IVS.mdl` 안 `generic_IVS/CarMaker/IVS_Control` 서브시스템에 구현.
멀티레이트 Stateflow 스케줄러(`Chart`, 10/50/100 ms)가 function-call로 각 모듈을 구동.

## 아키텍처 (파이프라인)
```
Read CM Dict → Ego[tx,ty,rz,v,sRoad]
ObjectManager(상대속도) → MapManager(route graph) → ModeManager(supervisor)
 → PathPlanning → TrajectoryPlanning → LateralController ∥ LongitudinalController
 → Safety/ActuatorManager → Write CM Dict
```
| 모듈 | 역할 | 레이트 |
|---|---|---|
| Perception_ObjectManager | 29 Traffic Object → `Obstacle_Info`(전역 x,y,yaw) | base |
| ObjectManager | ego 상대 위치+**상대속도**(유한차분) → `ObsFeat` | 10 ms |
| MapManager | route graph 로컬라이즈 → `MapCtx` | 50 ms |
| ModeManager | 차선/추월/톨/주차 미션 FSM → `Mode` | 50 ms |
| PathPlanning | 목표차선/주차분기 경로 → `Path` | 100 ms |
| TrajectoryPlanning | lookahead + 목표속도(ACC) → `Trajectory` | 100 ms |
| LateralController | Pure Pursuit → `Steer` | 10 ms |
| LongitudinalController | 속도 PD → `Accel` | 10 ms |
| Safety/ActuatorManager | AEB + saturation, 단일 writer | 10 ms |

## I/O 계약 (CarMaker)
- IN: `Car.Fr1.tx/ty/rz`, `Car.v`, `Car.Road.sRoad`, `Traffic.T00..T28.tx/ty`
- OUT: `Car.CFL.rz_ext`+`Car.CFR.rz_ext`(조향), `AccelCtrl.DesiredAx`(가속)
- ⚠ 차량 AccelCtrl 게인·TestRun 환경은 **무수정** (DesiredAx 명령만 사용)

## 파라미터 (모델 워크스페이스, 자립형)
`LANE_XY`(3차선 일반화), `cruise_lane=1`, `v_cruise=13`, `v_park=2.78`(10km/h), `WB=2.9`,
`toll_lane=1`(=1차선 x≈−176.17), `toll_x/y`(waypoint 기반), `branch/entrance`(주차 분기·입구), `park_goal=[35,−30,120°]`(뒷범퍼).

## 미션 흐름 (검증됨)
주행 13 m/s → **톨게이트 1차선 무정지 통과** → lane −1 분기 → 주차 입구 2.78 m/s 접근 → 정지 →
`Mode.parking_ready=1` (Phase 2 주차 핸드오프).

## 알려진 이슈 / 진행 중
- **충돌 미해결**: .erg 조밀 분석(ego-객체 최소거리, dt=0.01) 결과 **이동 교통차량과 충돌 잔존**(최소 ~0.07 m).
  원인: 곡선에서 선행차 감지 실패 + 늦은 제동. closest-approach 기반 ACC/AEB로 재구현했으나 **효과 미검증** — 튜닝 중.
- **(4) 주차장 정적차량 occupancy 회피** + Phase 2 주차 = 보유 주차 알고리즘 통합 예정.

## 분석 방법 (충돌 정량화)
`SaveMode save` + `OutQuantsAdd {Car.Fr1.* Traffic.Tnn.tx/ty}` → 실행 → `cmread(.erg)` →
매 시점 ego-29객체 최소거리 계산 → 충돌 이벤트(거리<임계) 검출. (이전엔 듬성 폴링만 해 충돌을 놓쳤음 — 정정함)
