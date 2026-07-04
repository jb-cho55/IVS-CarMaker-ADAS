# IVS 자율주행 — 세션 핸드오프 (2026-06-06 갱신 #2, 커밋 e8fa332)

## 프로젝트
CarMaker(CM4SL) MATLAB R2024a + CarMaker 14.1.2. 미션: 다차로주행 13m/s → 톨게이트 1차선 무정지 → 주차분기 → 입구정지 → 후진주차(pp_parking).
제약: `AccelCtrl.DesiredAx` + `Car.CFL/CFR.rz_ext` + `DM.SelectorCtrl` 명령만. 차량게인/TestRun 수정불가(단 **OutputQuantities 계측 로깅은 가능** — 거동 불변). 범용(좌표 하드코딩 금지, 인지기반).
모델: `src_cm4sl/generic_IVS.mdl` 안 `CarMaker/IVS_Control`. EML: objmgr/modemgr/mapmgr/pathplan/trajplan/latctrl/lonctrl/safety + Parking(parking_wrap→pp_parking).

## ⚠️ 핵심 교훈 (다음 세션 필독)
1. **진짜 충돌 측정 = `Sensor.Collision.Vhcl.Fr1.Count`** (IPGMovie 빨강의 근거, Vehicle Kia_EV6의 Collision Sensor COL00). `Data/Config/OutputQuantities`의 `DStore.Quantities.normal:`에 로깅됨. 시계열 max>0이면 그 런 충돌. **overlap(rect_gap gap<0.5)은 근사** — 폐기. `dev/analysis/ivs_eval.m`이 `nColl` 주지표 + `collObj`(충돌시점 최근접객체).
2. **stochastic 압도적**: 동일구성도 런마다 충돌 1~6개·입구도달·정지율 크게 변동. **N≥8 필수**. (leaddet가 N=4에서 충돌-free 1/4였으나 N=8에서 0/8 — 운이었음.) 미세튜닝 효과가 노이즈에 묻힘.
3. **충돌0 vs 완주 = trade-off**: 보수적이면 충돌0이나 미완주(멈춤/데드락), 진행하면 전방추돌. 동시 달성이 핵심 난제.
4. **모든 traffic 충돌 = ego 전방 일렬추돌** (앞차에 범퍼간≈0, xr≈+4.7, yr≈0). 고속추격+늦은인지+정지거리 부족. ego는 −4 풀제동해도 이미 늦음(v13 정지거리 21m > 확보 범퍼간).
5. **T22(분기) ego-커플링**: 멈춤=데드락(영구), 추종=충돌. 횡 dodge 무효(T22 따라옴). 근본 딜레마. (이전 "후미추종" 진단은 overlap 오판 — 진짜충돌은 전방추돌.)

## 이번 세션 (커밋 e8fa332) — 입구도달 0→100%
- ✅ 측정인프라: Sensor.Collision 로깅 + ivs_eval 진짜충돌 기반 재작성.
- ✅ **safety 전방AEB**(APPROACH AEB를 `xr>0` 한정): 후방 T22 데드락 직접해소 → 입구 0→75%, 재랩 50→0%.
- ✅ **trajplan 정지거리차간**(lead following `d_safe=4+0.12*v^2`): 톨충돌 일부완화.
- ✅ **modemgr 톨감속**(`toll_lock`시 `vset=7`): 톨게이트 충돌 해소 → **입구도달 100%, 재랩 0**.
- 결과(N=8): 충돌-free 0/8(잔존), 입구 100%, 데드락·톨충돌·재랩 해소.

## 충돌 지도 (현재 = tollslow)
- 톨게이트: 해소됨.
- 주행초기~분기: T26/T20(톨직후 t68-70), T28(t62), T22(분기 t86-96 전방추돌). 전부 전방추돌.
- 입구/주차: 정적·도로객체(근접 traffic 없음 d>37), v~0.5 주차기동 중 → parking_wrap 경로 문제.

## 실패 실험 (롤백됨, 교훈)
- **longsafe**(safety 범퍼강제동 + lead d_safe 작게=3+0.6v): 과제동→입구0%. d_safe는 정지거리 기반이어야(거꾸로면 악화).
- **leaddet**(lead 탐지 dmin 1.6→2.2): 옆차선 오탐으로 T17 주행초기(t33-36) 충돌 유발, N8 충돌-free 0/8.
- **fixB dodge**(trajplan APPROACH 횡dodge, `dev/trajplan_dodge.m`): T22가 전방추돌이라 횡회피 무효.

## 잔존 (우선순위)
1. **충돌0**(최난제): 톨직후 T26/T20(ua<0.5 lead 강화 여지) + 분기 T22(ua>0.5 lead+creep 필요, 딜레마) + 주행초기 T28. **N≥8 검증 필수**, trade-off(보수성↔완주) 균형. 일부는 물리/구조 한계(고속추격, 능동방해).
2. **주차충돌**: 입구 도달 후 정적·도로객체. parking_wrap(pp_parking) 경로/실행 재보정. 전진park 직각슬롯 기하 불가.

## 작업 재개
```matlab
cd('C:\Users\gmkk6\Desktop\last_dance\02_Carmaker_project\Practice_sample\src_cm4sl');
addpath(genpath('dev')); cmenv; load_system('generic_IVS'); CM_Simulink;   % GUI
% 평가(진짜충돌): R=ivs_eval(8,160,'tag')  — N≥8 필수. nColl=충돌수, collObj=객체.
% EML 수정: rt=sfroot(); c=rt.find('-isa','Stateflow.EMChart');  해당(i).Script=새코드;
%           set_param('generic_IVS','SimulationCommand','update')   % 컴파일
```
하네스 지표: **충돌-free 비율(nColl==0) 주지표**, 입구도달, 재랩(toll==1), 정지율. **변동성 때문에 N≥8 필수.**
미커밋 측정: dev/analysis/leaddet*.mat(실패 실험).

## 메모리 (영구)
`~/.claude/.../memory/`: parking-exec-layer-not-planner, ivs-mission-collisions-relap, ivs-scenario-stochastic, **ivs-collision-measurement**.
