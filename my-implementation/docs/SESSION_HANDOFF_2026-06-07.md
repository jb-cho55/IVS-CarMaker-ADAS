# IVS 자율주행 — 세션 핸드오프 (2026-06-07)

이전 핸드오프(`../SESSION_HANDOFF.md`, 커밋 e8fa332)에 이어지는 작업 기록. **다음 세션은 이 문서로 이어가면 됨.**

## 프로젝트 / 미션 / 제약
- CarMaker(CM4SL) MATLAB R2024a + CarMaker 14.1.2. 모델 `src_cm4sl/generic_IVS.mdl` 내 `CarMaker/IVS_Control`. **모델 FixedStep=0.001(1ms)**.
- 미션: 다차로 13m/s → 톨 1차선 무정지 → 주차분기 → **주차장 입구 정지** → **후진주차**(pp_parking).
- 출력 제약: `AccelCtrl.DesiredAx` + `Car.CFL/CFR.rz_ext` + `DM.SelectorCtrl` 만. TestRun·차량게인 수정불가(OutputQuantities 로깅만 허용). 범용(인지기반).
- EML 차트(Stateflow, .mdl 내장): objmgr/modemgr/mapmgr/pathplan/trajplan/latctrl/lonctrl/safety + Parking(parking_wrap→pp_parking). pp_* 로직은 `dev/day4_5/`의 .m (ple/day7에서 가져옴).

## ⚠️ 근본 제약 (필독)
- **AccelCtrl PI 게인 P=0.05 / I=0.001 — 수정 절대 금지(사용자 지시)**. 종방향이 매우 sluggish: 명령한 감속/속도를 제때 못 냄 → 정밀 정지·하드제동 불가. **모든 잔존 난제의 근원**. 대응: 늦은 하드제동 대신 **이른 완만한 속도조절 / 회피 / feedforward**.
- 차량 제어가 정밀하지 않음 = 위 게인 때문(컨트롤러 로직 자체는 이상 plant에선 정상). 주차 정밀도 floor ~1~1.5m.
- **DM.SelectorCtrl 기어는 정상**: Fin_Gear ST=[0.001 0](1ms), 후진 −1 연속유지 확인. 기어는 문제 아님.

## 이번 세션 성과
### 1) 주행 traffic 충돌 거의 해결 (baseline 0/8 → 전체미션 4/5 collision-free)
적용한 ID기반 수술적 수정(전부 거동 개선):
- **T17 곡선 추돌** → trajplan에 **ID speed-match**(idx17~29, ego-프레임 코리도 `xr<32,|yr|<ycor`; `ycor=2.4`, v<8이면 4.5). 곡선서 횡-offset lead를 잡아 추종.
- **T26 톨큐(lane3 정지차)** → modemgr **toll_lane 조기 커밋**(`toll_lock = dtoll<60`, vset=7은 dtoll<25만). lane1로 일찍 이동해 lane3 큐 회피.
- **T22 분기 블로커** → trajplan **approach dodge**(idx23, `xrd∈(-8,22), |yrd|<5`, lookahead 횡 4.5m 시프트). 영구 정지 블로커를 옆으로 통과.
- **고속차(T26/27/28) 컷인** → modemgr 차선변경 가능판정에 **후방 고속접근 반영**(`xr<0 && vxr>3 && (xr+1.6*vxr)>-6` → blocked).

### 2) 주차 — pp_parking 작동시킴 (동결 → 후진주차)
- **근본원인 발견·해결**: `pp_obstacles.m`가 **day7 장애물 좌표**(IVS 아님)였음 → occ_map(PARK_OCC)에 엉뚱한 장애물 → PLAN 실패/오충돌. **실제 IVS 정적차 16대(T00~T15)로 교체** + `cfg.holdout=0`(슬롯 빈공간) + **PARK_OCC 재생성**.
- **후진 오버슈트 해결**: v_emax 가드가 reverse 부호로 never fire → 후진 2.75m/s 폭주. pp_parking에 **적응형 캡 `abs(ego_v)>abs(v_des)+0.25`** 추가(슬롯 근처 creep 허용).
- margin: **map(장애물) 0.5, 차량 0**(pp_collision은 actual footprint만). v_fwd 0.8(느린 staging).

## 🔴 KEY 정정 + 현재 OPEN 이슈 (다음 세션 1순위)
- **주차 목표 정정**: `park_goal = (9, -6, -π/2)` [주차장 윗줄 왼쪽 빈슬롯, 남향]. (이전 (21.4,-44.3,π/2)는 day7 잔재 — 틀림. 모델 워크스페이스에 (9,-6,-π/2)로 이미 저장됨.)
- **OPEN: 주차 항법 배회(=데드락)**. 입구(6,-37)에서 슬롯(9,-6)까지 **~30m 주차장 내부 항법**이 필요한데, pp_parking은 **슬롯 근처 짧은 기동용**이라 sluggish 실행으로 **배회→충돌**(gear=D 유지, 후진 도달 못 함, d2slot 20~32에서 못 좁힘).
  - **해결 방향**: 핸드오프(입구)를 **슬롯 가까운 통로**(슬롯 남쪽 aisle, 예 ~(9~15, -10~-12))로 옮겨, 주행/approach로 슬롯 근처까지 간 뒤 **pp_parking은 짧은 후진만** 하게. 즉 APPROACH 경로/entrance를 슬롯 근처로 연장하거나, 항법은 driving이 하고 pp_parking은 최종 후진만.
  - 참고: 오프라인 PLAN(`pp_hybrid_astar` 입구→(9,-6))은 ok=1, np=84로 경로는 존재. 문제는 **긴 경로의 실행(추종)**.
- **입구 정지 처리(이번 세션 적용)**: trajplan은 `sf>0.5`(stop) 시 lookahead를 정면 고정(헤딩 유지), modemgr는 `ev<0.3 && ~yawing`일 때 park 래치. 단 hold-heading이 정지 전 ego를 약간 동쪽으로 드리프트시켜 핸드오프가 (11,-38)서 일어남 → 재검토 필요.

## 현재 모델 상태 (저장됨)
- generic_IVS.mdl 저장 완료. park_goal=(9,-6,-π/2), PARK_OCC=IVS16대(margin0.5), 위 모든 EML/pp_* 수정 반영.
- pp_cfg.m: margin=0.5, holdout=0, v_fwd=0.8, v_rev=0.4. pp_parking.m: 적응형 후진캡. pp_obstacles.m: IVS 16대.

## 평가 / 재개 방법
```matlab
cd('C:\Users\gmkk6\Desktop\last_dance\02_Carmaker_project\Practice_sample\src_cm4sl');
addpath(genpath('dev')); cmenv; load_system('generic_IVS'); CM_Simulink;   % GUI 연결
% 평가: R=ivs_eval(N,tcap,'tag'). N≥5(검증), 주차포함시 tcap=260. nColl=진짜충돌(Sensor.Collision.Count).
%   ⚠ ivs_eval.m의 슬롯좌표 gx=21.4,gy=-44.3 → (9,-6)로 갱신해야 주차측정 정확.
% EML 수정: rt=sfroot; c=rt.find('-isa','Stateflow.EMChart'); 해당.Script=새코드;
%           set_param('generic_IVS','SimulationCommand','update')  % 컴파일
% pp_* 수정 후: clear functions; PARK_OCC 재생성 필요시:
%   BASE=pp_generate_map([],[],[]); FULL=pp_add_obstacle(BASE,[],[]);
%   assignin(get_param('generic_IVS','modelworkspace'),'PARK_OCC',FULL);
% 단일 주차런: cmguicmd('LoadTestRun IVS_Final_Project'); set_param start; (폴링); stop; cmread(.erg)
```
- 충돌 진단: 최신 .erg에서 `Sensor_Collision_Vhcl_Fr1_Count` max>0 + 충돌프레임 최근접객체. 정적차(T00~T15)는 erg 미로깅이라 좌표 하드코딩으로 footprint 체크(rect_gap).

## 다음 세션 NEXT STEPS (우선순위)
1. **주차 항법(OPEN)**: 핸드오프를 슬롯(9,-6) 근처 통로로 — APPROACH/entrance 재설계. 그래야 pp_parking이 짧은 후진으로 슬롯 진입. (현재 30m 배회가 최대 막힘.)
2. ivs_eval 슬롯좌표 (9,-6)로 갱신 후 전체미션 N≥5 재측정(주행+주차 collision-free + 슬롯도달).
3. 주행 잔존: 초기 고속차(T26/27/28) 가끔 충돌(stochastic ~1/5). yield 강화 여지.
4. 랩타임: v_fwd=0.8 등 보수설정이 시간↑. collision-0 확보 후 속도 최적화.

## 메모리 (영구, ~/.claude/.../memory/)
- accelctrl-gains-do-not-modify (P=0.05/I=0.001 고정)
- ivs-parking-occmap-was-day7 (PARK_OCC가 day7→IVS교체, 후진주차 작동)
- parking-exec-layer-not-planner, ivs-collision-measurement, ivs-scenario-stochastic, ivs-mission-collisions-relap
