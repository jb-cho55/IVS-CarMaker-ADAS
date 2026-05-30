# Day4_5 주차 개발 노트 / 백업 (WORK_PROGRESS)

> 최종 갱신: 2026-05-30. parking 브랜치에 백업됨.
> 이 문서 = 제어 인터페이스 지식 + 시스템 구조 + 핵심 발견 + 현재 전략 정리본.

---

## 0. TL;DR (현재 상태)
- **구동 인터페이스 확정**: User 모드 + `AccelCtrl.DesiredAx`(종방향, 양수=기어방향 구동/음수=브레이크) + `DM.SelectorCtrl`(+1 D/-1 R/-9 P). DVA 아님. 페달(VC.Gas/Brake) 금지(강사).
- **🔄 재시작 결정(사용자)**: anti-windup 등으로 너무 복잡해짐 → **복잡한 로직 전부 제거, 최소 단순 제어부터 하나씩 쌓기**.
  - 강의 방식: 방향=SelectorCtrl, 종방향=desired_ax 단순 제어. 그게 핵심.
  - `Parking_simple.m` = 최소 버전(단순 속도 P + selector + 조향 + 정지). 복잡 버전은 `Parking_simple_antiwindup_backup.m`에 보존.
- **플래너(RRT*+CCRS)는 보류**(넓은 맵 hang 버그). `Parking.m`은 parking 브랜치에 보존.
- **핵심 학습(보존)**: 후진을 desired_ax로 '매끈하게'는 구조적 한계 — AccelCtrl이 전진 전용이라 후진에서 windup(아래 ★ 참조). 단순 제어로 가면 후진은 펄스(끊김)가 날 수 있음 = 정상. 실제 주차는 후진이 짧아 덜 문제.

## ▶ 재시작 플랜 (단순 제어 → 하나씩 쌓기)
1. ✅ **최소 컨트롤러 검증 완료** (`Parking_simple.m`): 전진→정지, **짧고 살살한 후진**→정지→주차(P). 급발진 없음, 조향 wobble 줄임.
2. ✅ 검증된 **제어 레시피** 확립:
   - 종방향: `a_desr = KP·(v_des−|v|)` (속도 P) + **상승률 제한(slew: 가스 완만/브레이크 자유)** = surge 완화.
   - 후진: **속도상한 낮게(V_MAX_R=0.3)** + 후진은 짧게(REV_DIST) → windup 최소.
   - 방향: `DM.SelectorCtrl`(1/−1/−9), 정지 게이트로 기어 전환.
   - 조향(현재): 목표점 점추종(전진 게인 0.8/후진 0.35 + deadband) — **임시**.
3. ✅ **플래너 부착 완료 + 배포**: `Parking.m`(RRT*+CCRS + Stanley + compute_v_des) 의 종방향을 검증 레시피로 교체.
   - 종방향: 기존 PD+launch → **속도 P(KP_LON=0.8) + slew-limit(AX_SLEW_UP=0.05)**, 후진 V_TGT_MAX=0.3/compute_v_des V_MAX_REV=0.3 (살살).
   - 조향: **Stanley 경로추종**(cross-track) — 점추종 대신 정밀 추종.
   - 맵 [4,48]. VC.Gas/Brake 비활성. 배포 46097 chars(F5_WRITE_DONE).
   - 오프라인 검증(test_integrated_move): path_len=153, 차량 3.97m 이동, 기어 D↔R, t=0.82s 후진 시작, max 0.30m/s. ✓
4. ✅ **CarMaker 라이브: 목표까지 경로추종 완주 확인!** 전진 접근 + 후진 cusp으로 스폿 진입, 급발진/큰 이탈 없이 완료. 기어 cusp 간격 0.5→0.15s 단축 적용.
   - 백업: 단순 컨트롤러 `Parking_simple.m` 보존.

## ⚠️ 남은 이슈: 초반 스폰 클립 (소프트, 완주엔 지장 없음)
- 증상: 시작 직후 차가 가드레일을 **스침**(soft). 차는 회복하고 **완주함**.
- 원인: `day4_5_final_parking_only` 스폰이 **Global (5.5,−36.5), Orientation 180°(서향)** → 차 길이 4.7m라 **front가 lot 서쪽벽(x=4) 밖 3.2m**. 초반 back-out 첫 ~3m 동안 앞부분이 경계 밖(가드레일).
- 깔끔한 수정이 막힌 이유: 스폰 자세를 바꾸면(동향/동쪽이동) **인터프리터 RRT*가 행/매우 느림**(150s↑). (5.5,서향)만 목표를 일찍 찾아 ~14s에 풀리는 "운 좋은" 자세. 즉 플래너가 **시작 자세에 민감**.
- 판단: 완주에 지장 없으므로 **현재는 수용**. 깔끔히 없애려면 **플래너 robust화/가속**(better goal-bias, N_ITER 조정, 시드 튜닝, 또는 Simulink Coder로 컴파일) = 별도 작업. 또는 실제 진입로가 x<4 주행가능인지 확인(맵 과보수 가능성).

---

## 1. ⭐ 제어 인터페이스 (가장 중요 — 하드원 지식)

### 차량 설정 (`Data/Vehicle/Kia_EV6`, L624~)
```
VehicleControl.0.Kind = AccelCtrl      % 종방향은 AccelCtrl가 담당
AccelCtrl.DesrAccelFunc = User         % ★ User! (DVA 아님)
AccelCtrl.p = 0.001 ; AccelCtrl.i = 0.1  % i=0.1로 낮춤(급가속 억제) → 페달 천천히 쌓임
```
- **실측 결론**: 이 CM4SL 셋업에선 **`User` 모드라야 `AccelCtrl.DesiredAx` write가 차를 구동**한다.
  - DVA로 바꾸면 수동 주입(=1.0)에도 **무반응** → DVA는 이 환경에서 안 먹음. (한 번 DVA로 바꿨다가 원복함)
  - User 모드 + DVA 뷰어에서 `AccelCtrl.DesiredAx` 수동 set → **차 움직임 확인** = 구동 체인 정상.

### 우리가 쓰는 출력 (Day4_5_Scenario_1.slx → CarMaker)
| 신호 | CarMaker quantity | 의미 |
|---|---|---|
| desired_ax | `AccelCtrl.DesiredAx` (Write CM Dict19) | 종방향 가속도 명령 [m/s²]. **+=차체전진가속, −=감속/브레이크**. 0=현속 유지 |
| selector_ctrl | `DM.SelectorCtrl` (Write CM Dict17) | 기어: **+1=D, −1=R, −9=P, 0=N, 2=M** |
| steer_fl/fr | `Car.CFL/CFR.rz_ext` (Write CM Dict14/15) | 앞바퀴 조향각 [rad] |
| (vc_gas/brake) | VC.Gas/Brake (Dict18/16) | **Commented=비활성**. 절대 쓰지 않음(인터페이스 단일화) |

### 주의 (제약)
- **Vehicle Data Set(Kia_EV6) 수정 금지** (i=0.1 등 그대로). TestRun 수정 금지(`Driver.Long.Active=1` 그대로).
- 기어 변경은 **정지 상태에서만**(주행 중 D↔R 금지). 표준: 속도<0.12에서 N틱 게이트 후 전환.
- 좌표는 전부 **리어범퍼 중심**.
- 화면 기어 표시는 실제 기어와 무관 → **속도(Vhcl.v)로 판단**.

---

## 2. 시스템 구조
- **실행 모델 = `generic_IVS`** (`src_cm4sl/generic_IVS.mdl`). 안의 `CarMaker/Subsystem/Day4 & 5/Scenario 1` = **Subsystem Reference → `Day4_5_Scenario_1.slx`** (우리 작업 대상).
  - ⚠️ 디스크 수정 반영하려면 generic_IVS **`bdclose all`→재오픈** 필요.
  - generic_IVS에 EnablePort/Trigger 없음 → 서브시스템 항상 실행.
- **occupancy grid**(`generate_map_`): 고정 전역 프레임 **x∈[0,100], y∈[-100,0]**, RES=0.5, 200×200. `map_boundary`는 그 안의 주행가능 폴리곤만 정의.

### Day4_5_Scenario_1.slx 차트
| chart | 블록 | 역할 | 상태 |
|---|---|---|---|
| 131 | MATLAB Function1 | `generate_map_` | 활성 |
| 140 | MATLAB Function2 | `add_obstacle_` | 활성 |
| **109** | **MATLAB Function5** | **주차 컨트롤러(배포 대상)** | **활성** |
| 86/64/96 | MATLAB Function/3/4 | path_planner/중복 | 비활성(Commented) |
- **배포 대상 = MF5(chart_109)**. 입력 8개(ego_x/y/yaw/vx, Start_Point, Finish_Point, goal_yaw, occ_map), 출력 9개(desired_ax, steer_fl/fr, path_x/y/len_dbg, selector_ctrl, vc_gas/brake).
- 스코프: `Scope`=path_len_dbg, `Scope1`=path_y_dbg, `Scope2`=path_x_dbg.

### 현재 시나리오 상수 (.slx, 검증된 값으로 복원됨)
- Start_Point (5.5, −36.5) / Finish_Point (17, **−7**) / goal_yaw −pi/2 / **map_boundary [4,48]×[−46.8,−4]**.

---

## 3. 핵심 발견 (lessons learned)
1. **User 모드가 정답, DVA 아님** (위 §1). 과거 "안 움직임"의 한 원인은 VC.Gas/Brake를 끄고 desired_ax에 의존했는데 액추에이션 모드 이해가 틀렸던 것.
2. **플래너 무한루프(hang)**: 맵을 [2.5,49.5]로 넓히면 `rrt_ccrs_plan` 첫 호출이 안 끝남 → MF5 첫 스텝이 영영 안 끝남 → **desired_ax가 아예 안 나옴**. [4,48]에선 14초에 풀림. (넓은 자유공간 샘플링/start-corridor/RS 엔드게임 어딘가의 종료조건 버그로 추정)
3. **stay_put**: `path_len≤2`(경로 못 찾음)면 `v_des=0 → desired_ax=0`. 진단 시 `Scope`(path_len) 보면 됨.
4. **핸드오프 시작점 (1.819,−36.641)** 은 x<4라 넓은 맵 필요 → 플래너 hang 버그를 먼저 잡아야 통합 가능.

---

## 4. ▶ 현재 전략: 플래너 분리, 전진/후진 제어 먼저
**목표**: 플래너 없이 **전진해서 목표점에 정지 → 기어 R → 후진해서 목표점에 정지** 를 확실히 성공시킨다. 그다음 플래너를 붙인다.
- 구현: `Parking_simple.m` (planner-free 2-phase 컨트롤러, MF5와 동일 시그니처).
- 검증 항목: ①전진 가속·감속·정지 ②D→R 기어 전환 ③후진 가속·감속·정지.
- 배포: `Parking.m`(RRT*) 대신 `Parking_simple.m`을 MF5에 배포해 테스트. RRT* 버전은 git(parking 브랜치) + 디스크에 보존.

---

## 5. 파일 맵
| 파일 | 내용 |
|---|---|
| `Parking.m` | RRT*+CCRS 플래너 + Stanley + PD 종 (풀 버전, 보류) |
| `Parking_simple.m` | ★ planner-free 단순 전진/후진 컨트롤러 (현재 작업) |
| `generate_map_.m` / `add_obstacle_.m` | 점유격자 |
| `setup_final_parking_test.m` | 시나리오 상수/TestRun 구성 |
| `Day4_5_Scenario_1.slx` | 실제 모델(차트 임베드) |
| `WORK_PROGRESS.md` | (이 문서) |

## 6. 변경 이력 (요약)
- DVA로 변경했다가 **User로 원복**(DVA 무효 확인). 백업 `Kia_EV6.bak_before_DVA_20260530`.
- Finish_Point −11.7 → **−7** 복원. map [2.5,49.5] → **[4,48]** 복원(플래너 hang 회피).
- Parking.m 후진 파라미터를 전진성공 값으로 복원.
- parking 브랜치 푸시 + PR #7.

## 7. TODO
- [x] `Parking_simple.m` 작성 + 오프라인 검증(전진→정지→D/R전환→후진→정지 OK) + **MF5 배포 완료**.
- [ ] **CarMaker 라이브 검증**(전진/후진 실제 동작 + 후진 desired_ax 부호 확정). ← 현재 여기
- [ ] 성공 후 RRT* 플래너 재부착(넓은맵 hang 버그 수정 포함).
- [ ] FP 통합(`parking_scenario_fcn.m`).

## ★ desired_ax 부호 규약 (실측 확정 — 매우 중요)
- **`AccelCtrl.DesiredAx` 양수 = 기어방향 구동(가스), 음수 = 브레이크.** 방향은 기어(DM.SelectorCtrl)가 결정.
  - 전진(D) + 양수 → 앞으로 가속 ✓
  - 후진(R) + 양수 → 뒤로 가속 ✓  (← 그래서 `REV_AX_SIGN=+1`)
  - 어느 기어든 음수 → 브레이크.
- 실측 근거: 후진에 음수를 줬더니 **전혀 안 움직임(=브레이크)**. 양수를 줘야 후진함. (차체기준 −x 가설은 틀렸음.)
- 기존 RRT* Parking.m의 "양수=기어방향" 규약이 **맞았다**.
- **후진 끊김/급발진의 진짜 근본원인 = AccelCtrl 적분기 WINDUP (실측 확정)**:
  - DVA 뷰어로 후진 중 DesiredAx=1.0 set: 한 번=무반응, 두 번째=급발진. = windup.
  - 메커니즘: AccelCtrl PI error = DesiredAx − ActualAx(차체기준). 후진은 가스→뒤로가속→ActualAx **음수** → error=(+)−(−)=**점점 커짐** → 적분 무한누적 → 방출=급발진. (전진은 가스→ActualAx양수→error 닫힘→안정.)
  - 결론: **AccelCtrl은 전진용**. 후진을 DesiredAx로 매끄럽게 못 끔(양수 지속=windup서지 / 음수=브레이크 / 중간차단=start-stop).
  - 대응 1(현재): **anti-windup 히스테리시스 후진** — 느림=짧은가스, 정속=코스트(0,적분안감김), 과속/서지=브레이크(적분 방출). V_MAX_REV=0.4로 낮춤.
  - 대응 2(대안): ~~후진만 VC.Gas/Brake~~ → **강사님이 페달 제어 금지** 확정. desired_ax로만 풀어야 함.
  - 추가 통찰(실측): 후진 stall의 원인은 **desired_ax=0 코스트**였음 — 정속에서 0을 주면 드래그(차체기준 +가속)를 error로 읽어 AccelCtrl이 브레이크→정지. → **후진에선 0 절대 금지**. 약한 가스(0.18) 항상 유지 + 과속 시 강브레이크(-0.8)로 windup 방출. (0.18=stall방지+windup천천히 / 0.30=급발진). V_MAX_REV=0.3.
  - ⚠️ 이 테스트의 후진은 ~30m로 비현실적으로 길어 windup 누적이 큼. **실제 주차는 후진이 짧아** windup 덜 쌓임 → 플래너 단계에선 덜 문제될 수 있음.

## 8. Parking_simple 검증 노트
- 오프라인(직선, 플랜트 lag): fwd (0,0)->(10,0) x=10.016 정지, D->R 전환, rev ->(0,0). 로직 OK.
- 라이브 확인 시 후진이 안 가거나 반대로 가면 → `REV_AX_SIGN=-1.0` (후진 가속 부호), 조향 반대면 `REV_STEER_SIGN` 부호 변경.
- 플래너 없음 → **첫 스텝 14초 프리즈 없음**, **hang 없음**. 단 장애물 회피 없음(제어 검증 전용).
