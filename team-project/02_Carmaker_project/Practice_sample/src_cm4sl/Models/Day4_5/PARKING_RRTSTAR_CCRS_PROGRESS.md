# RRT* + CCRS 후진주차 — 작업 진행 상황

> 작업 추적/참조용 단일 소스. 매 단계마다 여기 갱신한다.
> 최종 업데이트: 2026-05-29 (인터페이스 통일: VC.Gas/Brake 폐지 → desired_ax+DM.SelectorCtrl 제어로 전환·배포)

## 0. 목표

YouTube `DLjeuGgDcTM` 데모처럼 **RRT\* + CCRS(Continuous-Curvature Reeds-Shepp)** 로
후진주차를 구현한다.

- 1차 개발/검증: **Day4_5 Simulink** (`Day4_5_Scenario_1.slx`, Parking 블록) +
  `day4_5_final_parking_only` 시나리오.
- 최종 통합: **Final Project** — `parking_scenario_fcn.m`(Dev D)만 수정, FP `.slx`는 건드리지 않음.
  완료 후 팀 GitHub(`ChungRyeung/26HL_IVS_ADAS`)에 PR.

## 1. 좌표/차량 규약 (중요)

- **모든 좌표(에고 start, 장애물, 목표)는 차량 뒷범퍼 중앙**. 차체는 heading(+) 방향으로 local-x ∈ [0, L] 전개.
- 주차장 크기 ≈ **40 × 40 m**.
- EV6: `EGO_W=1.9`, `EGO_L=4.7`, `REAR_AXLE_FROM_REAR_BUMPER=0.95`, 최대 조향 ≈ 0.5 rad.
  - 최소 회전반경 R ≈ WHEELBASE/tan(0.5) ≈ **5.3 m** → κ_max ≈ **0.19 /m**.
- Occupancy grid: 200×200, RES=0.5, X∈[0,100], Y∈[-100,0]
  (col=floor((x-XMIN)/RES)+1, row=floor((YMAX-y)/RES)+1).

## 2. 재사용할 기존 코드 (그대로 가져옴)

출처: `Parking_block_7out_for_existing_slx.m` (= 모델 Parking 챗 미러).

- **제어기** `control_with_shift_delay_local(ego_v, target_v, steer_cmd, dir_cmd)` (L1411~):
  → `vc_selector(+1 D / -1 R)`, `vc_gas`, `vc_brake`. 정지대기 후 기어전환(speed<0.12, 50 cycle),
  과속(brake 0.35)/저속(gas 0.18 R·0.24 D)/creep(gas 0.06 R·0.09 D) 페달 로직.
- **횡제어** Stanley + `compute_v_des`(경로기하 기반 속도프로파일: 정지거리·곡률·기어변경 lookahead).
- **출력 와이어링**: out1→VC.Gas, out6→VC.Brake, selector→VC.SelectorCtrl, steer_fl/fr→front wheel angle.
- **경로캐시** (L30~130): persistent `path_x/y/yaw/dir/path_len` + replan 조건
  (빈경로 / 목표이동>0.2 / REPLAN_PERIOD / goal-zone 재latch). 한 번 계획 후 매 스텝 재생.

→ **교체 대상은 플래너뿐**: `two_stage_parking_plan`(Hybrid A* + RS) → RRT*+CCRS.

## 3. 알고리즘 설계 (RRT* + CCRS)

상태공간 X = (x, y, θ) (뒷범퍼 pose). 자전거모델 pivot = 뒷차축.

### 3.1 CCRS — 연속곡률 Reeds-Shepp (연결/스티어 함수)
- 일반 RS는 직선↔원호 경계에서 곡률이 불연속(0↔κ_max) → 추종 시 조향 점프.
- CCRS는 그 경계에 **클로소이드(Euler spiral) 전이**를 삽입해 곡률을 연속으로 만든다.
  - sharpness σ = 최대 곡률변화율, 클로소이드 길이 δ = κ_max/σ.
  - 원호 진입/이탈에 clothoid-in / arc / clothoid-out (CC-turn 프리미티브).
- 전진/후진은 RS word의 direction 세그먼트로 자연 처리 → 후진주차에 적합.

**채택한 tractable 변형 (codegen·정확성 우선):**
완전 해석적 CC-RS(48 word × 클로소이드)는 codegen에 과도 → 다음 2단계로 분리:
1. RRT* 트리 edge/goal 연결 = **RS 해석解**(차량 feasible, 길이=cost).
2. 최종 경로에 **클로소이드 CC-스무딩 패스** = 곡률 불연속(heading-rate 점프) 지점에
   κ_max·σ 제한 클로소이드 전이 삽입 → 추종 가능한 연속곡률 경로(CCRS 특성) 확보.
ref 논문 합성 결론과 일치(“CC-RS면 C² 확보, 무거운 NMPC 스무딩 불필요”).

**구현 현황(v1):** 1번(해석적 RS)은 완성·검증 통과. 정준 RS word를
`CSC(LSL,LSR) + CCC(LRL)` 3개 base × **4 대칭(timeflip·reflect)** = 12 후보로 생성하고,
세그먼트 **부호 길이 = 진행방향**(+전진/−후진)이라 후진 기어 라벨이 정확히 나온다
(`rs_best_word`/`rs_try`/`rs_LSL|LSR|LRL`/`rs_interp`). 2번(클로소이드 CC-스무딩 패스
`smooth_ccrs`)은 **v2로 보류** — 추종기(Stanley)가 곡률 점프를 흡수하는지 본 뒤 필요시 추가.

### 3.2 RRT* (전역 탐색)
- goal-biased 샘플링(목표 pose 직접연결 시도 비율 ↑).
- nearest/near = CCRS 거리(길이+후진 패널티) 기준.
- steer = RS 해석解 연결, edge 전구간 충돌검사.
- **충돌모델 = half-width inflated plan_map + 종방향 중심선 샘플링**(빠름):
  맵을 `inflate_r`만큼 팽창시키면 ego를 중심선(local-x −0.35~L+0.45)만 샘플해도 됨.
  - `inflate_r = EGO_W/2 + 0.25 + 0.30`(=**1.50 m**). 마지막 0.30은 **이산화 쿠션**:
    장애물 래스터가 셀 경계에서 ~0.25 m 작게 잡히고, chamfer 거리변환이 ~0.10 m 과대평가 →
    쿠션 없이 inflate_r=half_w(1.20)면 풋프린트 모서리가 장애물을 최대 ~0.40 m 파고듦(검증으로 확인).
  - 쿠션을 0.40(inflate 1.60)까지 올리면 통로가 막혀 goal 미연결 → **0.30이 최적 절충**.
- rewire: 반경 `RN_MAX=6 m`(40 m 주차장은 노드가 빨리 차서 작은 반경이 충분+빠름) 내 비용 개선 시 재연결.
- 노드/반복 예산 고정: `MAX_NODES=2500`, `N_ITER=5000`, `EXTRA_AFTER_GOAL=800`. 경로는 캐시되므로 1회 계획비용만 중요.

### 3.3 후진 stall 진입
- 목표 직전은 CCRS의 직선 후진(S, dir=-1)이 정확히 stall 중심선에 정렬되도록 goal 연결을 우선.

### 3.4 개발 전략 (검증된 워크플로 답습)
1. **오프라인 .m** 으로 full-double 구현 → matplotlib plot 로 기하 검증.
2. 검증 후 **codegen-safe**(고정배열, %#codegen) 포팅 → Day4_5 챗에 write-back.
3. 저장모델에서 재추출 re-run 으로 end-to-end 검증(이전 파킹수정과 동일).

## 4. 시나리오 데이터 (day4_5_final_parking_only)

**`setup_final_parking_test.m`(권위 소스, L44-67) 기준 — 확정:**
- start = **[5.5, -36.5]** (뒷범퍼, yaw 180°)
- goal  = **[17.0, -7.0]**, goal yaw = **-π/2**
- map boundary = [4,-4; 4,-46.8; 48,-4; 48,-46.8]
- traffic size = [1.97; 4.47] (W; L), 장애물 T00..T20, Road=day7_final.rd5
- ※ `FINAL_PARKING_TEST.md`의 (35,-30, 2π/3)은 **구버전**. .m 파일이 권위.

### 4.1 참고문헌 분석 결과 (ref PDF 5종)
- 5종 모두 **Hybrid A* + 최적화(NMPC)** 중심. RRT*·CC-RS 직접구현은 없음(인용만).
- 차용할 구체값: 뒷차축 자전거모델, R_min≈5.5m, κ_max≈0.18/m, δ_max≈0.45~0.5rad,
  goal bias 15%, 충돌샘플 ≤0.2m, d_safe 0.1~0.3m, 장애물 inflate 0.1m.
- 충돌검사: OBB-SAT 또는 point-outside-rect 부등식 — 기존 Fix B 풋프린트 검사로 충분.
- RS는 RRT* edge cost·goal 연결 테스트로 사용(논문 heuristic h1과 동일 역할).
- 클로소이드/CC-turn 식은 논문에 없음 → Fraichard-Scheuer CC-Steer 방식으로 구현.

## 5. 진행 로그

| 일자 | 단계 | 내용 |
|---|---|---|
| 2026-05-29 | 설계 | 재사용 코드 확정(제어기+캐시), RRT*+CCRS 설계 수립, 본 md 생성. ref 주차논문 분석 완료. goal=(17,-7,-90°) 확정. |
| 2026-05-29 | 주의 | 저장소 standalone `add_obstacle_.m`(L57-58,182-183)는 Fix A 미반영 구버전 → 오프라인은 수정본 맵빌더 사용. (.slx 챗에는 Fix A 반영됨) |
| 2026-05-29 | 구현 | 오프라인 `rrt_ccrs_plan.m` 작성. 초기 단순 CSC는 후진 원호 표현 불가 → goal 미연결(plen=2). **정준 해석적 RS(12 word)로 재작성** 후 연결 성공. |
| 2026-05-29 | 디버그 | 연결되나 풋프린트가 T02를 최대 0.40 m 침범. 원인=충돌모델 이산화 과소팽창(래스터 0.25+chamfer 0.10). `inflate_r` 쿠션 0.30 추가(→1.50), 풋프린트 샘플 0.45→0.35, edge 서브샘플 13→4. |
| 2026-05-29 | **검증 통과** | start(5.5,−36.5,180°)→goal(17,−7,−90°). **plen=187, goal 정확 도달(거리 0.000 m·yaw 0.00°), cusp 4, 풋프린트 충돌 0**(MATLAB 래스터 + Python SAT 독립검증 일치, 최소 여유 0.29 m). 계획 111 s(인터프리터, 챗 컴파일 시 대폭 단축). |
| 2026-05-29 | 후진제어 진단 | 사용자 보고: 실제 CarMaker 후진이 약함. 매뉴얼 분석 → 모델은 `DM.Gas/Brake/SelectorCtrl` 직접 write(Manual-mode 등가, 후진 지원). 약함 원인 = **후진 gas < 전진**(저속 0.18<0.24, creep 0.06<0.09) + EV6 `GasInterpret TrqZero=-10Nm` 오프셋 + BEV `RegBrake`(<2km/h)·`RegDrag`(<5km/h) 둘 다 Active=1 → 주차속도(<0.6m/s)에서 회생드래그가 후진토크 잠식. 전진은 더 큰 gas로 극복, 후진은 stall. |
| 2026-05-29 | 후진제어 수정 | 사용자 선택 **"제어기 후진가속 보강"**(공유 Kia_EV6 차량파일·회생 미변경). `Parking.m`(권위 9-out 소스) `control_with_shift_delay_local`: 후진 저속 gas **0.18→0.30**, creep **0.06→0.15**(전진 0.24/0.09보다 크게 → 회생드래그 극복). `_7out_` 미러 동기화. `apply_rev_gas.m`(최소 write-back, Parking 챗만 갱신·add_obstacle Fix A 보존)으로 `Day4_5_Scenario_1.slx` 기록 완료(`REVERSE_GAS_PATCH_DONE`). **CarMaker 실차 후진 검증은 사용자 실행 대기.** |
| 2026-05-29 | **근본원인 규명** | 사용자 보고: 실차에서 장애물 회피 0 + 후진 0. 확인 결과 **챗에는 아직 구 플래너(`two_stage_parking_plan` Hybrid A*+RS)가 돌고 있고 RRT*+CCRS 미통합**. + `diag_real.m`로 실제 경계(x∈[4,48]) 재현 → **start(5.5,−36.5,180°) 풋프린트가 서벽(x=4)을 침범**(앞범퍼 x=0.35, 풋프린트 중심선 20/30 샘플 occupied). 차가 벽에 코박고 주차된 상태→후진탈출 시나리오인데 플래너가 start 부근 모든 edge를 충돌로 버려 **경로 0** → 회피·후진 둘 다 불능. 이전 오프라인 성공(plen=187)은 경계를 x_min=0으로 넓혀 우회한 것이라 실제 맵 미반영이었음. |
| 2026-05-29 | start-corridor 수정 | 플래너에 `carve_start_corridor`(plan_map에서 start 후진 back-out 통로를 free; D=EGO_L+3.5, 폭=half_w+0.6; 실장애물과 ≥9m 떨어져 안전) 추가. `run_rrt_real.m`(실제 경계 + 동일 corridor를 raw occ에도 carve해 검증 일관) 작성. |
| 2026-05-29 | **실경계 검증 통과** | 실제 경계 `[4,−4;4,−46.8;48,−4;48,−46.8]`로 **plen=185, goal 정확(0.000m·0.00°), cusps=2, 첫 방향=−1(후진), EXACT-FOOTPRINT PASS(0/185)**. 계획 85.8s(인터프리터). → RRT*+CCRS 챗 통합 준비 완료. |
| 2026-05-29 | **잘못된 블록에 기록(버그)** | 챗에 `function [desired_ax` 마커가 **3개**(MATLAB Function3·4·5). `apply_rev_gas.m`/`write_parking_chart.m`가 first-match로 **MATLAB Function3(비활성)** 에 기록 → 실차 무변화. VC.Gas/Brake/Selector(Dict18/16/17)에 배선된 **활성 블록은 MATLAB Function5**(`patch_vc_pedal_model_api.m` L28 확인). 사용자 보고(회피 0·후진 0)의 직접 원인 = 수정이 라이브 블록에 없었음. |
| 2026-05-29 | **활성 블록 통합 완료** | `write_parking_F5.m`(chart.Path=='…/MATLAB Function5'로 타깃)으로 통합 Parking.m을 **MATLAB Function5에 기록**. 재추출 확인: F5 rrt=1·carve=1·gas030=1(len 50126→39841), MATLAB Function2(add_obstacle_ Fix A)·Function1(generate_map_) 무손상. 배선 검증: out7→Dict17(selector)·out8→Dict18(gas)·out9→Dict16(brake)·out1/2/3→Dict19/14/15 보존. **CarMaker 실차 검증 사용자 대기**(첫 replan 수 초 소요, 이후 캐시). |
| 2026-05-29 | **실차 후진주차 성공(잔여 충돌)** | 통합 후 실차에서 후진주차 동작 시작. 사용자 보고: **전 구간 뒤쪽이 균일하게 살짝 충돌**. |
| 2026-05-29 | 기준점 검증 | 런타임 ego는 `Car.Fr1.tx/ty/rz`+`Car.v`. 프레임 오프셋 가설 검증 위해 `Kia_EV6` 데이터셋 확인: 뒷바퀴 x=0.95·앞바퀴 x=3.85(WB 2.90), **Fr1 원점이 뒷축보다 0.95m 뒤 = EV6 뒤 오버행 ≈ 뒷범퍼**. → **Car.Fr1 ≈ 뒷범퍼, 프레임 버그 없음**(0.95 미사용 상수가 맞음). 가설 기각. |
| 2026-05-29 | 잔여충돌 원인/수정 | "직선 포함 전 구간 균일 + 뒤쪽" = 곡선의존 추종오차가 아니라 **후진 정상상태 횡오차가 빠듯한 측면여유(오프라인 최소 0.29m) 잠식 → 뒤 코너 클리핑**. 수정: `EGO_WIDTH_SAFETY_MARGIN` 0.25→**0.35**(inflate_r 1.50→1.60, 측면여유 0.55→0.65m), `EGO_REAR_SAFETY_MARGIN` 0.35→**0.50**, `V_MAX_REV` 0.6→**0.45**(후진 추종 강화). Parking.m+standalone 동기. |
| 2026-05-29 | **재검증+재배포** | 넓힌 여유로 오프라인: plen=208·goal 정확(0.000m·0.00°)·후진우선·EXACT-FOOTPRINT PASS(0/208)·cusps=4. 통합 end-to-end OK. `write_parking_F5.m`로 **MATLAB Function5 재기록**(len 39846, Function1/2 무손상). **CarMaker 실차 재검증 사용자 대기.** |
| 2026-05-29 | 헤딩 무관 요구 명확화 | 사용자: 전진/후진을 미리 정하는 게 아니라 **마지막 날 강사가 최종 헤딩앵글을 주면 플래너가 그 헤딩에 맞춰 경로 생성**해야 함. → 플래너를 **heading-agnostic**으로 만드는 게 목표. goal 포즈(Constant15·finish_point)는 강사 입력으로 두고 플래너만 임의 헤딩 대응. |
| 2026-05-29 | dual-side staging | 기존 stage 휴리스틱은 후진 전용(`gx+4.5cos(gyaw)` 한쪽). 전진(+90°) goal에선 통로 반대(벽)쪽을 유도→연결 실패(plen=2). 수정: **양쪽 stage 점**(stgx_a=+gyaw, stgx_b=−gyaw, stage_d 4.5→3.0)을 50/50 샘플링해 통로 방향이 항상 유도됨. standalone+Parking.m 동기. |
| 2026-05-29 | 탐색예산 상향 | 전진주차는 통로 자유폭 ~3m < 회전반경 5.13m으로 기하학적으로 빡빡(다중 절환 필요) → `N_ITER` 5000→**20000**, `EXTRA_AFTER_GOAL` 800→**2000**, `MAX_NODES` 2500→**8000**. 예산은 상한일 뿐 goal 발견 시 조기종료 → 쉬운(후진) 케이스는 비용 증가 없음. |
| 2026-05-29 | **전진주차 오프라인 검증** | 전진 nose-in goal(17,−11.7,+90°): **plen=191·goal 정확(0.000m·0.00°)·마지막 dir=+1(전진)·EXACT-FOOTPRINT PASS(0/191)·cusps=9**. 계획 506s(인터프리터, 빡빡한 통로로 절환 多). 후진 재검증: 22.4s·plen=161·cusps=4·PASS(조기종료로 더 빨라짐). 통합 `Parking()` 전진 end-to-end: **INTEGRATED_FWD_OK**(591s·plen=191·goal 정확). |
| 2026-05-29 | **heading-agnostic 배포** | `write_parking_F5.m`로 robust 플래너(dual staging+상향 예산)를 **MATLAB Function5 재기록**(41665 chars, Function1/2 무손상). 모델 goal 포즈는 현행 후진(−π/2,17,−7) 유지 — 강사가 Constant15·finish_point로 헤딩 지정 시 플래너가 자동 대응. **주의: 빡빡한 헤딩(전진 등)은 첫 플래닝이 수 분 소요·절환 多.** |
| 2026-05-29 | 인터페이스 통일 피드백 | 사용자 피드백: 인터페이스 통일 위해 **VC.Gas/VC.Brake 미사용**, 대신 `desired_ax`(소수 가속, m/s²)+`DM.SelectorCtrl`(-1 후진)+목표속도로 제어. 확인: TestRun `Driver.Long.Active=1`·`IPGOperator 1` → IPGDriver 종방향 활성, `AccelCtrl.DesiredAx`가 표준 가속입력. **VC.Gas/Brake write는 드라이버 페달을 덮어씀**(VC.Gas=0 써도 페달 0으로 강제) → 반드시 write 비활성(comment) 필요. |
| 2026-05-29 | 제어법 재작성 | `control_with_shift_delay_local` **전면 재작성**: 속도크기 PD를 기어부호로 투영 `desired_ax = active_selector*(KP·(|v|_tgt−|v|)+KD·d_lpf)`, KP=1.5/KD=0.20/ALPHA=0.6, AX∈[−2.5,1.0]. 기어는 정지대기(|v|<0.12, 50cyc) 후 `DM.SelectorCtrl` 커밋(§6.5.8 v=0 페달부호반전 급발진 회피). `vc_gas/brake`는 0 유지(레거시 출력). 포인트매스 단위테스트(`test_ctrl_law.m`) 부호·정지 안정 확인, 통합 `test_integrated2.m` INTEGRATED_PARKING_OK. |
| 2026-05-29 | 배선 변경 probe | `probe_wiring.m`: -batch(CarMaker4SL 미해결)에서 `set_param('Commented','on')` **가능**(readback=on), `set_param('xname',...)` **불가**(마스크 param이 미해결 링크에 있음) → xname은 XML 직접편집 필요 확인. |
| 2026-05-29 | **desired_ax 인터페이스 배포** | (1) `write_parking_F5.m`로 신규 제어법 **MATLAB Function5 기록**(43338 chars, 스테일 `vc_gas=0.30` assert를 신규 법칙 assert로 교체). (2) 동 스크립트로 **Dict18(VC.Gas)·Dict16(VC.Brake) Commented=on**(API). (3) **Dict17 xname VC.SelectorCtrl→DM.SelectorCtrl** 을 .slx zip의 system_root.xml에 **.NET ZipArchive in-place 편집**(SID 4014만 스코프, 다른 VC.SelectorCtrl 2개 무손상). 검증: load_system OK·Dict18/16 Commented=on·Dict17 active·F5 신규법칙 확인. out1→AccelCtrl.DesiredAx(Dict19) 기존 유지. **CarMaker 실차 검증 사용자 대기.** |
| 2026-05-29 | **후진 미동작 → 부호 수정** | 사용자 보고: 실차 후진 안 됨. 원인 = `desired_ax = active_selector*ax_mag`로 **후진 시 음수(=브레이크) 명령**을 보냄 → 차 정지. AccelCtrl.DesiredAx는 **기어 진행방향 기준**(양수=기어방향 가속, 음수=제동)이고 방향은 DM.SelectorCtrl이 이미 결정 → **기어부호 투영 제거**(`desired_ax = ax_mag`, 후진도 양수로 가속). 사용자 요청대로 **gentle launch ~0.1**(`AX_START=0.10`, `AX_RISE=0.50` m/s³ rate-limit, 제동은 즉시) 추가. 단위테스트 확인: 후진 sel=-1·ax 0.11→cruise(+)→brake(-)→정지, 전진 대칭. `write_parking_F5.m` 재배포(44573 chars), Dict17 xname=DM.SelectorCtrl save 라운드트립 보존 확인. **CarMaker 실차 재검증 사용자 대기.** |
| 2026-05-29 | **급발진 재발 → PID 재설계** | 사용자 보고: 후진은 되나 또 급발진. 원인 = **미분킥**(`e_d=(e-e_prev)/DT`가 목표속도 step(0→cruise, 기어전환 직후)에서 폭발 → 명령이 cap으로 슬램). 사용자 요청대로 **목표속도 PID** 재작성: ①**미분을 측정값 기준**(`-KD·d|v|/dt`, 킥 제거) ②**적분항+anti-windup**(clamp `EI_MAX=1.0`, 포화시 unwind)+**적분분리**(`abs(e)<=EI_BAND=0.20`에서만 적산 → 가속구간 windup 오버슈트 방지) ③gentle 튜닝 `KP=0.8/KI=0.25/KD=0.10`, accel cap `AX_MAX=0.40`, **launch `AX_START=0.03`**·`AX_RISE=0.20`, 목표속도 하드캡 `V_TGT_MAX=0.60`. 단위테스트(`test_ctrl_law.m`): 후진 ax 0.03→peak 0.23(<0.40)·오버슈트 28%→4%·매끈한 제동, 전진 대칭. **급발진 0**. `write_parking_F5.m` 재배포(45927 chars), Dict18/16 Commented=on·Dict17 xname=DM.SelectorCtrl 보존 확인. **CarMaker 실차 재검증 사용자 대기.** |
| 2026-05-29 | **후진 급가속 근본수정 — 기어별 gentle 분기** | 사용자: "여전히 후진 급가속, 근본적으로 제어방법이 잘못". **근본원인 = 이중 적분기(cascade)**: 우리 PID→`desired_ax` → CarMaker 내부 AccelCtrl **PI(p=0.001,i=1.0≈순수적분기)** → VC.Gas/Brake. 큰 명령(전진 `AX_MAX=0.40`/목표 0.60)을 주면 내부 적분기가 가스를 세게 적분, **전진은 파워트레인이 따라오나 후진(EV 회생/크립 지연)은 과적분→일시 방출=급가속**. **부호는 정상**(후진도 양수=가속, 음수=제동, 실험확인)—문제는 크기·동특성. 전진은 잘 되므로 **건드리지 않고 후진(active_selector<0) 구간만** softening: `V_TGT_MAX=0.30`·`KP=0.35/KI=0.08/KD=0.08`·`AX_MAX=0.12`·`AX_MIN=-0.60`·`AX_RISE=0.06`. 단위테스트(`test_ctrl_law.m`): 후진 **peak ax 0.40→0.082 m/s²**, 매끈한 가속·제동, 급가속 0; 전진 peak 0.270(불변). `write_parking_F5.m` 재배포(**47118 chars**) MATLAB Function5, Dict18/16 Commented=on. **CarMaker 실차 재검증 사용자 대기.** |
| 2026-05-29 | **42.9km/h 충돌 — 근본원인: AccelCtrl DVA 미설정** | 실차 실행 결과 **속도계 42.9 km/h·기어1로 주행 중 벽/차 충돌**(스크린샷). 우리 제어가 차량 종방향을 **전혀 못 잡음**(우리 캡은 전진 0.6/후진 0.3 m/s). 추적: `Vehicle/Kia_EV6` L624-625 = `VehicleControl.0.Kind="AccelCtrl"`·**`AccelCtrl.DesrAccelFunc = User`**. `User` 모드는 desired 가속도를 사용자 C-함수에서 받음 → 우리가 쓰는 DVA 양 `AccelCtrl.DesiredAx`를 **안 읽음**. 게다가 `src/CM_Vehicle.c`엔 `Set_UserDesrAccelFunc` 미등록(`VC_Register_AccelCtrl()`만) → User AccelCtrl은 전 시나리오에서 **사실상 무효**, 모두 IPGDriver(`Driver.Long.Active=1`)로만 주행해 옴. VC.Gas/Brake까지 주석 처리한 상태라 Day4_5는 오버라이드 0 → IPGDriver 폭주. **수정: `Kia_EV6` L625 `User`→`DVA`** (refman 4565: DVA여야 `AccelCtrl.DesiredAx` 읽음). 다른 날은 DesiredAx 미기록→NOTSET→AccelCtrl 비활성(동일거동)이라 안전. **CarMaker 재실행으로 속도캡(≈2km/h 이하) 적용·충돌 해소 확인 필요.** |
| 2026-05-29 | **제어 전면 검토 → 외부적분 제거(PD화)** | 사용자: "근본적으로 뭔가 잘못, 전체 제어코드 검토·수정". 재해석: 메시지2·3·5의 후진 급가속은 **전부 DVA OFF 상태**에서 발생 → 우리 desired_ax는 차량에 한 번도 연결된 적 없었고 PID 튜닝이 거동을 못 바꾼 게 당연(=근본원인=인터페이스 단절, 직전 DVA 수정으로 해소). 매뉴얼 재확인: `DM.SelectorCtrl=-1`=후진 정상(refman 17574), VehicleControl=AccelCtrl가 IPGDriver Gas/Brake를 가로채 DesiredAx로 재생성(4404) → DVA만 켜지면 동작. **2차 결함=이중 적분기**: 외부 PID(KI) → 내부 AccelCtrl PI(i=1.0, 적분기) 직렬 → windup·surge 구조적. **수정: 외부 루프 적분 제거 → PD화** `desired_ax=KP·e − KD·d\|v\|/dt`(미분=측정값, 킥 없음). 내부 PI가 `desired_ax=0`일 때 항속(드래그 보상) 담당 → 외부 적분 불필요. 기어별 게인: 후진 KP0.40/KD0.10/cap0.12/V0.30, 전진 KP0.70/KD0.12/cap0.40/V0.60. 단위테스트(내부 PI 1차지연 모델 포함): **후진 peak ax 0.095·최고속 0.287(오버슈트 0)**, 전진 peak 0.274·최고속 0.533. 배포 47095 chars. **CarMaker 재검증 대기.** |

## 6. 미해결/확인 필요

1. **계획 시간(런타임)**: 인터프리터 111 s. 챗은 codegen 컴파일이라 수십 배 빨라지나, 실시간 예산
   확인 필요. 필요 시 best-parent/rewire 후보를 공간 인덱스/k-NN로 제한하거나 N_ITER·EXTRA 축소.
2. **start 부근 경로 비효율**: 시작 직후 작은 전진 jog/wiggle(cusp 소비) 존재. 충돌은 없으나
   비효율 → start heading 정렬 개선/추가 rewire로 다듬기(검증엔 무영향).
3. **서서향(yaw 180°) start가 벽에 박힘**: 오프라인은 맵 경계를 x_min=0으로 넓혀 회피.
   챗 실제 경계는 x∈[4,48]이고, 기존 검증 시스템은 start를 충돌검사 없이 seed함 → 통합 시
   start 무검사 seed 또는 진입로 모델링으로 처리.
4. **CC 클로소이드 스무딩(v2)**: Stanley 추종이 RS 곡률 점프를 흡수하는지 본 뒤 필요 시 `smooth_ccrs` 추가.
5. **codegen 포팅 + 챗 통합**(작업 #9): 고정배열/`%#codegen` 유지, 캐시+제어기 보존, 저장모델 재추출 검증.

## 7. 오프라인 산출물 위치

- 플래너: `C:\Users\User\AppData\Local\Temp\park_rrt\rrt_ccrs_plan.m` (메인 deliverable)
- 하네스: `...\park_rrt\run_rrt.m` (맵빌드→계획→정확 풋프린트 재검증→CSV/JSON 덤프)
- 플롯/검증: `C:\Users\User\AppData\Local\Temp\park_plot\plot_path.py`, `diag_collisions.py`(SAT 독립검증)
- 결과 이미지: `...\park_plot\parking_path.png`
